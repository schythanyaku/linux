// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2014-2025 MediaTek Inc.
 * Copyright (c) 2025-2026 NVIDIA Corporation
 *
 * Tegra254 PCIe hotplug driver for NVIDIA DGX Spark
 *
 * Manages PCIe link power / hotplug for the CX7 endpoint using GPIO
 * interrupts and ACPI resources (cable detect, bring-up, tear-down).
 */

#include <linux/acpi.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pci_hotplug.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/uuid.h>

#define HP_PORT_MAX		3
#define HP_POLL_CNT_MAX		200
#define MAX_VENDOR_DATA_LEN	16
#define TEGRA254_HP_MMIO_REGION_COUNT	5	/* TOP, PROTECT, CKM, MAC Port 0, MAC Port 1 */
#define TEGRA254_HP_MIN_GPIO_COUNT	4	/* Minimum required: BOOT, PRSNT, PERST, EN */
#define PINCTRL_MAPPING_ENTRY_SIZE 5	/* dev_name, state, ctrl_dev, group, function */
/* Indices for pinctrl mapping entry strings */
#define PINCTRL_IDX_DEV_NAME	0
#define PINCTRL_IDX_STATE	1
#define PINCTRL_IDX_CTRL_DEV	2
#define PINCTRL_IDX_GROUP	3
#define PINCTRL_IDX_FUNCTION	4

/* Hardware timing requirements (in microseconds unless noted) */
#define TEGRA254_HP_DELAY_SHORT_US		10	/* Short delay for register writes */
#define TEGRA254_HP_DELAY_STANDARD_US	10000	/* Standard delay (10ms) */
#define TEGRA254_HP_DELAY_BUS_PROTECT_US	5000	/* Bus protection setup delay */
#define TEGRA254_HP_DELAY_PHY_RESET_US	3000	/* PHY reset delay */
#define TEGRA254_HP_DELAY_LINK_STABLE_MS	100	/* Link stabilization delay (ms) */
#define TEGRA254_HP_POLL_SLEEP_US		10000	/* Polling loop sleep interval */
/* BOOT FW ready poll: 10ms * 3000 ≈ 30s (CX7 firmware can take seconds) */
#define TEGRA254_HP_BOOT_POLL_MAX		3000

/* Bus protection stages to prevent PCIe core reset glitches */
#define BUS_PROTECT_INIT		0
#define BUS_PROTECT_CABLE_REMOVAL	1
#define BUS_PROTECT_CABLE_PLUGIN	2
#define BUS_PROTECT_CLEANUP		3

enum pcie_pin_index {
	PCIE_PIN_BOOT = 0,	/* Device boot status pin */
	PCIE_PIN_PRSNT,		/* Presence detection pin */
	PCIE_PIN_PERST,		/* PCIe reset pin */
	PCIE_PIN_EN,		/* Power enable pin */
	PCIE_PIN_CLQ0,		/* Clock request pin 0 */
	PCIE_PIN_CLQ1,		/* Clock request pin 1 */
	PCIE_PIN_MAX
};

/* BOOT pin state machine (drives when to call enable_slot) */
enum tegra254_hp_boot_state {
	BOOT_IDLE = 0,		/* No cable or not tracking */
	BOOT_POWERING_ON,	/* EN=1, waiting for BOOT high */
	BOOT_FW_LOADING,	/* BOOT went high, firmware loading (BOOT may go low) */
	BOOT_READY		/* BOOT low→high, firmware ready, call enable_slot */
};

struct pcie_port_info {
	int domain;
	int bus;
	int devfn;
};

struct rp_bus_mmio_top {
	u32 ctrl;
	u32 port_bits[HP_PORT_MAX];
	u32 update_bit;
};

struct rp_bus_mmio_protect {
	u32 mode;
	u32 enable;
	u32 port_bits[HP_PORT_MAX];
};

struct rp_bus_mmio_mac {
	u32 init_ctrl;
	u32 ltssm_bit;
	u32 phy_rst_bit;
};

struct rp_bus_mmio_ckm {
	u32 ctrl;
	u32 disable_bit;
};

struct rp_bus_mmio_info {
	struct rp_bus_mmio_top top;
	struct rp_bus_mmio_protect protect;
	struct rp_bus_mmio_mac mac;
	struct rp_bus_mmio_ckm ckm;
};

struct gpio_acpi_context {
	struct device *dev;
	unsigned int debounce_timeout_us;
	int pin;
	int wake_capable;
	unsigned long irq_flags;
	int valid;
	unsigned int connection_type;
};

struct tegra254_hp_dev;

/**
 * struct tegra254_hp_plat_data - Platform configuration data parsed from ACPI
 *
 * Platform-specific configuration parsed from ACPI devices:
 * - RES0 device (PNP0C02): PCIe configuration and MMIO register offsets via _DSD
 * - PEDE device (MTKP0001): Pinctrl mappings via _DSD
 */
struct tegra254_hp_plat_data {
	int port_nums;
	struct pcie_port_info ports[HP_PORT_MAX];
	u32 vendor_id;
	u32 device_id;
	int num_devices;
	struct rp_bus_mmio_info rp_bus_mmio;
	u32 ltssm_reg;
	u32 ltssm_l0_state;
	int pin_nums;
	struct pinctrl_map *parsed_pinmap;
};

struct tegra254_hp_discover_pcie_ctx {
	struct tegra254_hp_plat_data *pd;
	int *count;
	bool *retry;
};

struct tegra254_hp_gpio_ctx {
	struct gpio_desc *desc;
	struct gpio_acpi_context *ctx;
	struct tegra254_hp_dev *hp_dev;
};

struct acpi_gpio_parse_context {
	struct gpio_acpi_context *ctx;
	struct tegra254_hp_dev *hp_dev;
};

/* First-pass _CRS walk: only pin + controller name needed for desc lookup */
struct acpi_gpio_walk_context {
	struct device *dev;
	struct gpio_info {
		unsigned int pin;
		char resource_source[16];
	} gpios[PCIE_PIN_MAX];
	int count;
};

struct tegra254_hp_acpi_mmio {
	struct acpi_resource_fixed_memory32
	    mmio_regions[TEGRA254_HP_MMIO_REGION_COUNT];
	int count;
	struct device *dev;
};

struct tegra254_hp_mmio_runtime {
	void __iomem *top_base;
	void __iomem *protect_base;
	void __iomem *ckm_base;
	void __iomem *mac_port_base[HP_PORT_MAX];
};

/**
 * tegra254_hp_slot - Per-port hotplug slot for PCI core
 */
struct tegra254_hp_slot {
	struct hotplug_slot slot;
	struct tegra254_hp_dev *hp_dev;
	int port_idx;
	struct pci_dev *root_port;
};

/** Per-slot work to defer remove off disable_slot (avoids nesting with .remove) */
struct tegra254_hp_disable_work {
	struct work_struct work;
	struct tegra254_hp_dev *hp_dev;
	int port_idx;
};

/**
 * tegra254_hp_dev - Hotplug device structure
 *
 * ACPI resource sources:
 * - MMIO addresses: RES0 device (PNP0C02) _CRS, stored in mmio field
 * - GPIO resources: PEDE device (MTKP0001) _CRS, stored in pins field
 */
struct tegra254_hp_dev {
	struct tegra254_hp_gpio_ctx *pins;
	struct tegra254_hp_plat_data *pd;
	struct platform_device *pdev;
	int gpio_count;
	int boot_pin;
	int prsnt_pin;
	bool hotplug_enabled;
	spinlock_t lock;
	struct mutex slot_mutex;	/* protects slot ops and BOOT/PRSNT work */
	struct tegra254_hp_mmio_runtime mmio;
	struct gpio_device *gdev;
	struct notifier_block pci_notifier;
	bool pending_prsnt;		/* PRSNT pin fired, thread should schedule prsnt_work */
	struct tegra254_hp_slot slots[HP_PORT_MAX];
	struct tegra254_hp_disable_work disable_work[HP_PORT_MAX];
	struct work_struct prsnt_work;
	struct work_struct boot_work;
	struct work_struct probe_hp_work;	/* post-probe cable sync → slot HP */
	bool cable_present;
	enum tegra254_hp_boot_state boot_state;
	int slots_enabled_count;	/* 0 = hw down, >0 = hw up */
	bool pending_boot;		/* boot pin fired, thread should schedule boot_work */
	int last_boot_val;		/* previous BOOT pin value for state machine */
	struct completion boot_fw_ready; /* signaled when boot_work reaches BOOT_READY */
	bool boot_irq_waiter;		/* enable_slot waiting on boot_fw_ready */
};

/* ACPI _DSD device properties GUID: daffd814-6eba-4d8c-8a91-bc9bbf4aa301 */
static const guid_t device_properties_guid =
GUID_INIT(0xdaffd814, 0x6eba, 0x4d8c,
	  0x8a, 0x91, 0xbc, 0x9b,
	  0xbf, 0x4a, 0xa3, 0x01);

/**
 * tegra254_hp_parse_one_pinctrl_mapping - Fill one pinctrl_map from _DSD entry
 *
 * Each ACPI mapping is Package(5) of strings: dev_name, state, ctrl_dev,
 * group, function (Spark PEDE ASL format).
 */
static int tegra254_hp_parse_one_pinctrl_mapping(struct device *dev,
						 const union acpi_object *entry,
						 struct pinctrl_map *map,
						 int index)
{
	const char *strings[PINCTRL_MAPPING_ENTRY_SIZE];
	int l;

	if (entry->type != ACPI_TYPE_PACKAGE ||
	    entry->package.count != ARRAY_SIZE(strings)) {
		dev_err(dev,
			"Invalid pinctrl mapping entry %d: expected Package(%zu), got %s(count=%u)\n",
			index, ARRAY_SIZE(strings),
			entry->type == ACPI_TYPE_PACKAGE ? "Package" :
							   "non-Package",
			entry->type == ACPI_TYPE_PACKAGE ?
				entry->package.count : 0);
		return -EINVAL;
	}

	for (l = 0; l < ARRAY_SIZE(strings); l++) {
		if (entry->package.elements[l].type != ACPI_TYPE_STRING) {
			dev_err(dev,
				"Mapping entry %d element %d is not a string\n",
				index, l);
			return -EINVAL;
		}
		strings[l] = entry->package.elements[l].string.pointer;
	}

	map->dev_name = devm_kstrdup(dev, strings[PINCTRL_IDX_DEV_NAME],
				     GFP_KERNEL);
	map->name = devm_kstrdup(dev, strings[PINCTRL_IDX_STATE], GFP_KERNEL);
	map->type = PIN_MAP_TYPE_MUX_GROUP;
	map->ctrl_dev_name = devm_kstrdup(dev, strings[PINCTRL_IDX_CTRL_DEV],
					  GFP_KERNEL);
	map->data.mux.group = devm_kstrdup(dev, strings[PINCTRL_IDX_GROUP],
					   GFP_KERNEL);
	map->data.mux.function =
		devm_kstrdup(dev, strings[PINCTRL_IDX_FUNCTION], GFP_KERNEL);

	if (!map->dev_name || !map->name || !map->ctrl_dev_name ||
	    !map->data.mux.group || !map->data.mux.function) {
		dev_err(dev, "Failed to allocate memory for mapping %d\n",
			index);
		return -ENOMEM;
	}


	return 0;
}

/**
 * tegra254_hp_parse_pinctrl_config_dsd - Parse pinctrl configuration from PEDE device _DSD
 * @hp_dev: hotplug device
 *
 * Parses pin-nums and pinctrl-mappings from _DSD.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_parse_pinctrl_config_dsd(struct tegra254_hp_dev *hp_dev)
{
	struct acpi_device *adev;
	struct device *dev = &hp_dev->pdev->dev;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	const union acpi_object *mappings_pkg = NULL;
	const union acpi_object *dsd_pkg, *props_pkg = NULL;
	const union acpi_object *prop, *prop_value;
	const char *prop_name;
	struct pinctrl_map *pinmap;
	acpi_status status;
	u32 pin_nums = 0;
	int i, j, k;
	int ret;

	adev = ACPI_COMPANION(dev);
	if (!adev) {
		dev_err(dev, "Failed to get ACPI companion device\n");
		return -ENODEV;
	}

	status = acpi_evaluate_object_typed(adev->handle, "_DSD", NULL, &buffer,
					    ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "Failed to evaluate _DSD: %s\n",
			acpi_format_exception(status));
		return -ENODEV;
	}

	dsd_pkg = buffer.pointer;
	if (!dsd_pkg || dsd_pkg->type != ACPI_TYPE_PACKAGE) {
		dev_err(dev, "Invalid _DSD package\n");
		ACPI_FREE(buffer.pointer);
		return -EINVAL;
	}

	for (i = 0; i + 1 < dsd_pkg->package.count; i += 2) {
		const union acpi_object *guid = &dsd_pkg->package.elements[i];
		const union acpi_object *pkg =
		    &dsd_pkg->package.elements[i + 1];

		if (guid->type == ACPI_TYPE_BUFFER && guid->buffer.length == 16 &&
		    pkg->type == ACPI_TYPE_PACKAGE &&
		    guid_equal((guid_t *)guid->buffer.pointer,
			       &device_properties_guid)) {
			props_pkg = pkg;
			break;
		}
	}

	if (!props_pkg) {
		dev_err(dev,
			"Device Properties GUID package not found in _DSD\n");
		ACPI_FREE(buffer.pointer);
		return -EINVAL;
	}

	for (j = 0; j < props_pkg->package.count; j++) {
		prop = &props_pkg->package.elements[j];

		if (prop->type != ACPI_TYPE_PACKAGE ||
		    prop->package.count != 2 ||
		    prop->package.elements[0].type != ACPI_TYPE_STRING)
			continue;

		prop_name = prop->package.elements[0].string.pointer;
		prop_value = &prop->package.elements[1];

		if (!strcmp(prop_name, "pin-nums")) {
			if (prop_value->type == ACPI_TYPE_INTEGER)
				pin_nums = prop_value->integer.value;
		} else if (!strcmp(prop_name, "pinctrl-mappings")) {
			if (prop_value->type == ACPI_TYPE_PACKAGE)
				mappings_pkg = prop_value;
		}
	}

	if (pin_nums == 0) {
		hp_dev->pd->pin_nums = 0;
		ACPI_FREE(buffer.pointer);
		return 0;
	}

	if (!mappings_pkg) {
		dev_err(dev,
			"Missing required _DSD property: pinctrl-mappings\n");
		ACPI_FREE(buffer.pointer);
		return -EINVAL;
	}

	if (mappings_pkg->package.count != pin_nums) {
		dev_err(dev,
			"pinctrl-mappings count mismatch: expected %u, got %u\n",
			pin_nums, mappings_pkg->package.count);
		ACPI_FREE(buffer.pointer);
		return -EINVAL;
	}

	pinmap = devm_kcalloc(dev, pin_nums, sizeof(*pinmap), GFP_KERNEL);
	if (!pinmap) {
		ACPI_FREE(buffer.pointer);
		return -ENOMEM;
	}

	for (k = 0; k < pin_nums; k++) {
		ret = tegra254_hp_parse_one_pinctrl_mapping(
			dev, &mappings_pkg->package.elements[k], &pinmap[k], k);
		if (ret) {
			ACPI_FREE(buffer.pointer);
			return ret;
		}
	}

	hp_dev->pd->pin_nums = pin_nums;
	hp_dev->pd->parsed_pinmap = pinmap;
	ACPI_FREE(buffer.pointer);
	dev_dbg(dev, "Successfully parsed %u pinctrl mappings from ACPI\n",
		pin_nums);
	return 0;
}

/**
 * tegra254_hp_pinctrl_init - Register pinctrl mappings for the device
 * @hp_dev: hotplug device
 *
 * Parses pinctrl mappings from _DSD and registers them.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_pinctrl_init(struct tegra254_hp_dev *hp_dev)
{
	int ret;

	ret = tegra254_hp_parse_pinctrl_config_dsd(hp_dev);
	if (ret) {
		dev_err(&hp_dev->pdev->dev,
			"Failed to parse pinctrl configuration from ACPI: %d\n",
			ret);
		return ret;
	}

	if (!hp_dev->pd->pin_nums)
		return 0;

	ret = pinctrl_register_mappings(hp_dev->pd->parsed_pinmap,
					hp_dev->pd->pin_nums);
	if (ret) {
		dev_err(&hp_dev->pdev->dev,
			"Failed to register pinctrl mappings\n");
		return ret;
	}

	dev_dbg(&hp_dev->pdev->dev, "Registered %u pinctrl mappings\n",
		hp_dev->pd->pin_nums);
	return 0;
}

/**
 * tegra254_hp_pinctrl_remove - Unregister pinctrl mappings
 * @hp_dev: hotplug device
 */
static void tegra254_hp_pinctrl_remove(struct tegra254_hp_dev *hp_dev)
{
	if (!hp_dev->pd->pin_nums)
		return;

	pinctrl_unregister_mappings(hp_dev->pd->parsed_pinmap);
}

/**
 * tegra254_hp_change_pinctrl_state - Change pinctrl state
 * @hp_dev: hotplug device
 * @new_state: new pinctrl state name
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_change_pinctrl_state(struct tegra254_hp_dev *hp_dev,
				       const char *new_state)
{
	struct pinctrl *pinctrl;
	struct pinctrl_state *state;
	int ret;

	pinctrl = devm_pinctrl_get(&hp_dev->pdev->dev);
	if (IS_ERR(pinctrl)) {
		dev_err(&hp_dev->pdev->dev, "Failed to get pinctrl\n");
		return PTR_ERR(pinctrl);
	}

	state = pinctrl_lookup_state(pinctrl, new_state);
	if (IS_ERR(state)) {
		dev_err(&hp_dev->pdev->dev, "Failed to lookup state:%s\n",
			new_state);
		return PTR_ERR(state);
	}

	ret = pinctrl_select_state(pinctrl, state);
	if (ret) {
		dev_err(&hp_dev->pdev->dev,
			"Failed to select pinctrl state:%s\n", new_state);
		return ret;
	}

	return 0;
}

/**
 * tegra254_hp_reg_update_bits - Update specific bits in a register
 * @base: MMIO base address
 * @offset: Register offset
 * @mask: Bits to modify
 * @set: true to set bits, false to clear bits
 */
static inline void tegra254_hp_reg_update_bits(void __iomem *base, u32 offset,
					  u32 mask, bool set)
{
	u32 val = readl(base + offset);

	if (set)
		val |= mask;
	else
		val &= ~mask;

	writel(val, base + offset);
}

/**
 * tegra254_hp_toggle_update_bit - Toggle control register update bit
 * @base: MMIO base address
 * @ctrl_offset: Control register offset
 * @bits: Bits to set/clear before toggling update
 * @update_bit: Update bit mask
 * @set: true to set bits, false to clear bits
 *
 * Performs the sequence: modify bits, clear update bit, set update bit
 */
static void tegra254_hp_toggle_update_bit(void __iomem *base, u32 ctrl_offset,
				     u32 bits, u32 update_bit, bool set)
{
	tegra254_hp_reg_update_bits(base, ctrl_offset, bits, set);
	tegra254_hp_reg_update_bits(base, ctrl_offset, update_bit, false);
	tegra254_hp_reg_update_bits(base, ctrl_offset, update_bit, true);
}

/**
 * tegra254_hp_bus_protect_enable - Enable bus protection for a port
 * @dev: hotplug device
 * @port_idx: Port index
 */
static void tegra254_hp_bus_protect_enable(struct tegra254_hp_dev *dev, int port_idx)
{
	struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
	u32 port_bit = mmio_info->protect.port_bits[port_idx];

	tegra254_hp_reg_update_bits(dev->mmio.protect_base,
			       mmio_info->protect.mode, port_bit, true);
	tegra254_hp_reg_update_bits(dev->mmio.protect_base,
			       mmio_info->protect.enable, port_bit, true);
}

/**
 * tegra254_hp_bus_protect_disable - Disable bus protection for a port
 * @dev: hotplug device
 * @port_idx: Port index
 */
static void tegra254_hp_bus_protect_disable(struct tegra254_hp_dev *dev, int port_idx)
{
	struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;
	u32 port_bit = mmio_info->protect.port_bits[port_idx];

	tegra254_hp_reg_update_bits(dev->mmio.protect_base,
			       mmio_info->protect.enable, port_bit, false);
	tegra254_hp_reg_update_bits(dev->mmio.protect_base,
			       mmio_info->protect.mode, port_bit, false);
}

/**
 * tegra254_hp_ckm_control - Control clock module
 * @dev: hotplug device
 * @disable: true to disable clock, false to enable
 */
static void tegra254_hp_ckm_control(struct tegra254_hp_dev *dev, bool disable)
{
	struct rp_bus_mmio_info *mmio_info = &dev->pd->rp_bus_mmio;

	if (!dev->mmio.ckm_base)
		return;

	tegra254_hp_reg_update_bits(dev->mmio.ckm_base, mmio_info->ckm.ctrl,
			       mmio_info->ckm.disable_bit, disable);
}

/**
 * tegra254_hp_parse_mmio_resources - ACPI resource callback for parsing MMIO from _CRS
 * @ares: ACPI resource being processed
 * @data: pointer to tegra254_hp_acpi_mmio structure
 *
 * Returns: AE_OK to continue iteration, AE_ERROR on error
 */
static acpi_status tegra254_hp_parse_mmio_resources(struct acpi_resource *ares,
					       void *data)
{
	struct tegra254_hp_acpi_mmio *parsed = data;

	switch (ares->type) {
	case ACPI_RESOURCE_TYPE_FIXED_MEMORY32:
		if (parsed->count >= TEGRA254_HP_MMIO_REGION_COUNT) {
			dev_warn(parsed->dev,
				 "More than %d MMIO regions found in platform configuration device, ignoring extras\n",
				 TEGRA254_HP_MMIO_REGION_COUNT);
			break;
		}
		parsed->mmio_regions[parsed->count] = ares->data.fixed_memory32;
		parsed->count++;
		break;
	default:
		break;
	}

	return AE_OK;
}

/**
 * tegra254_hp_find_pcie_config_device - Find PCIe configuration device by HID
 *
 * Finds the ACPI device that provides PCIe configuration via _DSD properties
 * and MMIO resources via _CRS.
 *
 * Returns: acpi_device pointer on success (with reference), NULL on failure
 */
static struct acpi_device *tegra254_hp_find_pcie_config_device(void)
{
	return acpi_dev_get_first_match_dev("PNP0C02", NULL, -1);
}

/**
 * tegra254_hp_parse_pcie_config_dsd - Parse PCIe configuration from _DSD
 * @pdev: platform device
 * @pd: platform data to populate
 *
 * Parses PCIe MMIO register offsets, bit positions, port configuration, and PCIe device
 * identification from PCIe configuration device _DSD.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_parse_pcie_config_dsd(struct platform_device *pdev,
					struct tegra254_hp_plat_data *pd)
{
	struct acpi_device *config_adev;
	struct device *dev = &pdev->dev;
	u32 val, bit1;

	config_adev = tegra254_hp_find_pcie_config_device();
	if (!config_adev) {
		dev_err(dev,
			"Platform configuration device (PNP0C02) not found - _DSD is required\n");
		return -ENODEV;
	}

	if (!acpi_dev_has_props(config_adev)) {
		dev_err(dev,
			"Platform configuration device has no _DSD properties. Check DSDT.\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "mac-init-ctrl-offset", &val)) {
		dev_err(dev,
			"Missing required _DSD property: mac-init-ctrl-offset\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.mac.init_ctrl = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "mac-ltssm-bit", &val)) {
		dev_err(dev, "Missing required _DSD property: mac-ltssm-bit\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.mac.ltssm_bit = BIT(val);

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "mac-phy-rst-bit", &val)) {
		dev_err(dev,
			"Missing required _DSD property: mac-phy-rst-bit\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.mac.phy_rst_bit = BIT(val);

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "top-ctrl-offset", &val)) {
		dev_err(dev,
			"Missing required _DSD property: top-ctrl-offset\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.top.ctrl = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "top-update-bit", &val)) {
		dev_err(dev,
			"Missing required _DSD property: top-update-bit\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.top.update_bit = BIT(val);

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "top-port0-bit", &val)) {
		dev_err(dev, "Missing required _DSD property: top-port0-bit\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.top.port_bits[0] = BIT(val);

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "top-port1-bit", &val)) {
		dev_err(dev, "Missing required _DSD property: top-port1-bit\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.top.port_bits[1] = BIT(val);

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "protect-mode-offset", &val)) {
		dev_err(dev,
			"Missing required _DSD property: protect-mode-offset\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.protect.mode = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "protect-enable-offset", &val)) {
		dev_err(dev,
			"Missing required _DSD property: protect-enable-offset\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.protect.enable = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "protect-port0-bit", &val)) {
		dev_err(dev,
			"Missing required _DSD property: protect-port0-bit\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.protect.port_bits[0] = BIT(val);

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "protect-port1-bit", &val)) {
		dev_err(dev,
			"Missing required _DSD property: protect-port1-bit\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.protect.port_bits[1] = BIT(val);

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "ckm-ctrl-offset", &val)) {
		dev_err(dev,
			"Missing required _DSD property: ckm-ctrl-offset\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.ckm.ctrl = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "ckm-disable-bit0", &val)) {
		dev_err(dev,
			"Missing required _DSD property: ckm-disable-bit0\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "ckm-disable-bit1", &bit1)) {
		dev_err(dev,
			"Missing required _DSD property: ckm-disable-bit1\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->rp_bus_mmio.ckm.disable_bit = BIT(val) | BIT(bit1);

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "ltssm-reg-offset", &val)) {
		dev_err(dev,
			"Missing required _DSD property: ltssm-reg-offset\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->ltssm_reg = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "ltssm-l0-state", &val)) {
		dev_err(dev,
			"Missing required _DSD property: ltssm-l0-state\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->ltssm_l0_state = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "port-nums", &val)) {
		dev_err(dev, "Missing required _DSD property: port-nums\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	if (val == 0 || val > HP_PORT_MAX) {
		dev_err(dev,
			"Invalid _DSD property port-nums: %u (must be 1-%d)\n",
			val, HP_PORT_MAX);
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->port_nums = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "port0-domain", &val)) {
		dev_err(dev, "Missing required _DSD property: port0-domain\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->ports[0].domain = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "port0-bus", &val)) {
		dev_err(dev, "Missing required _DSD property: port0-bus\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->ports[0].bus = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "port0-devfn", &val)) {
		dev_err(dev, "Missing required _DSD property: port0-devfn\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->ports[0].devfn = val;

	if (pd->port_nums >= 2) {
		if (fwnode_property_read_u32
		    (acpi_fwnode_handle(config_adev), "port1-domain", &val)) {
			dev_err(dev,
				"Missing required _DSD property: port1-domain\n");
			acpi_dev_put(config_adev);
			return -EINVAL;
		}
		pd->ports[1].domain = val;

		if (fwnode_property_read_u32
		    (acpi_fwnode_handle(config_adev), "port1-bus", &val)) {
			dev_err(dev,
				"Missing required _DSD property: port1-bus\n");
			acpi_dev_put(config_adev);
			return -EINVAL;
		}
		pd->ports[1].bus = val;

		if (fwnode_property_read_u32
		    (acpi_fwnode_handle(config_adev), "port1-devfn", &val)) {
			dev_err(dev,
				"Missing required _DSD property: port1-devfn\n");
			acpi_dev_put(config_adev);
			return -EINVAL;
		}
		pd->ports[1].devfn = val;
	}

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "vendor-id", &val)) {
		dev_err(dev, "Missing required _DSD property: vendor-id\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->vendor_id = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "device-id", &val)) {
		dev_err(dev, "Missing required _DSD property: device-id\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->device_id = val;

	if (fwnode_property_read_u32
	    (acpi_fwnode_handle(config_adev), "num-devices", &val)) {
		dev_err(dev, "Missing required _DSD property: num-devices\n");
		acpi_dev_put(config_adev);
		return -EINVAL;
	}
	pd->num_devices = val;

	dev_dbg(dev, "Successfully parsed all required _DSD properties\n");

	acpi_dev_put(config_adev);
	return 0;
}

/**
 * tegra254_hp_parse_mmio_resources_from_acpi - Parse MMIO regions from _CRS
 * @dev: hotplug device
 * @parsed: pointer to parsed MMIO structure
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_parse_mmio_resources_from_acpi(struct tegra254_hp_dev *dev,
						 struct tegra254_hp_acpi_mmio
						 *parsed)
{
	struct acpi_device *config_adev;
	acpi_status status;
	int ret = 0;

	if (!dev || !dev->pdev)
		return -EINVAL;

	config_adev = tegra254_hp_find_pcie_config_device();
	if (!config_adev)
		return -ENODEV;

	parsed->count = 0;
	memset(parsed->mmio_regions, 0, sizeof(parsed->mmio_regions));

	status = acpi_walk_resources(config_adev->handle, METHOD_NAME__CRS,
				     tegra254_hp_parse_mmio_resources, parsed);
	if (ACPI_FAILURE(status)) {
		dev_err(&dev->pdev->dev,
			"Failed to walk platform configuration resources: %s\n",
			acpi_format_exception(status));
		ret = -ENODEV;
		goto out;
	}

	if (parsed->count < TEGRA254_HP_MMIO_REGION_COUNT) {
		dev_warn(&dev->pdev->dev,
			 "Expected %d MMIO regions from platform configuration device, found %d\n",
			 TEGRA254_HP_MMIO_REGION_COUNT, parsed->count);
		ret = -ENODEV;
		goto out;
	}

out:
	acpi_dev_put(config_adev);
	return ret;
}

/**
 * tegra254_hp_ioremap_named - Map one FIXED_MEMORY32 region
 */
static int tegra254_hp_ioremap_named(struct device *dev, u32 addr, u32 size,
				     void __iomem **out, const char *name)
{
	void __iomem *base;

	base = devm_ioremap(dev, addr, size);
	if (!base) {
		dev_err(dev, "Failed to map %s region (0x%08x)\n", name, addr);
		return -ENOMEM;
	}
	*out = base;
	return 0;
}

/**
 * tegra254_hp_map_mmio_resources - Map all MMIO regions from ACPI _CRS
 * @dev: hotplug device
 *
 * Firmware _CRS FIXED_MEMORY32 order is fixed: MAC Port 0, MAC Port 1,
 * TOP, PROTECT, CKM. Map MAC slots for configured ports, then the three
 * fixed regions.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_map_mmio_resources(struct tegra254_hp_dev *dev)
{
	struct platform_device *pdev = dev->pdev;
	struct device *d = &pdev->dev;
	struct tegra254_hp_acpi_mmio parsed = {.count = 0, .dev = d };
	void __iomem **fixed_bases[] = {
		&dev->mmio.top_base,
		&dev->mmio.protect_base,
		&dev->mmio.ckm_base,
	};
	static const char * const fixed_names[] = {
		"TOP", "PROTECT", "CKM",
	};
	/* MAC0/MAC1 always occupy CRS[0..1]; fixed regions start at CRS[2] */
	const unsigned int fixed_crs_base = 2;
	int mapped_count = 0;
	int ret;
	int i;
	u32 addr, size;

	ret = tegra254_hp_parse_mmio_resources_from_acpi(dev, &parsed);
	if (ret) {
		dev_err(d,
			"Failed to get MMIO regions from platform configuration device\n");
		return ret;
	}

	dev_dbg(d, "Found %d MMIO regions in _CRS, mapping...\n", parsed.count);

	for (i = 0; i < 2 && i < parsed.count; i++) {
		if (i >= dev->pd->port_nums)
			continue;

		addr = parsed.mmio_regions[i].address;
		size = parsed.mmio_regions[i].address_length;
		ret = tegra254_hp_ioremap_named(d, addr, size,
						&dev->mmio.mac_port_base[i],
						i == 0 ? "MAC Port 0" :
							 "MAC Port 1");
		if (ret)
			return ret;
		mapped_count++;
	}

	for (i = 0; i < ARRAY_SIZE(fixed_bases); i++) {
		unsigned int idx = fixed_crs_base + i;

		if (idx >= parsed.count)
			break;

		addr = parsed.mmio_regions[idx].address;
		size = parsed.mmio_regions[idx].address_length;
		ret = tegra254_hp_ioremap_named(d, addr, size, fixed_bases[i],
						fixed_names[i]);
		if (ret)
			return ret;
		mapped_count++;
	}

	if (!dev->mmio.top_base || !dev->mmio.protect_base ||
	    !dev->mmio.ckm_base ||
	    (dev->pd->port_nums >= 1 && !dev->mmio.mac_port_base[0]) ||
	    (dev->pd->port_nums >= 2 && !dev->mmio.mac_port_base[1])) {
		dev_err(d,
			"Required MMIO regions not mapped from ACPI _CRS (mapped %d)\n",
			mapped_count);
		if (!dev->mmio.top_base)
			dev_err(d, "  Missing: TOP\n");
		if (!dev->mmio.protect_base)
			dev_err(d, "  Missing: PROTECT\n");
		if (!dev->mmio.ckm_base)
			dev_err(d, "  Missing: CKM\n");
		if (dev->pd->port_nums >= 1 && !dev->mmio.mac_port_base[0])
			dev_err(d, "  Missing: MAC Port 0 (port_nums=%d)\n",
				dev->pd->port_nums);
		if (dev->pd->port_nums >= 2 && !dev->mmio.mac_port_base[1])
			dev_err(d, "  Missing: MAC Port 1 (port_nums=%d)\n",
				dev->pd->port_nums);
		dev->mmio.top_base = NULL;
		dev->mmio.protect_base = NULL;
		dev->mmio.ckm_base = NULL;
		for (i = 0; i < HP_PORT_MAX; i++)
			dev->mmio.mac_port_base[i] = NULL;
		return -ENODEV;
	}

	dev_dbg(d, "Successfully mapped all MMIO regions from ACPI _CRS\n");
	return 0;
}

/**
 * tegra254_hp_rp_bus_protect - Bus protection handler
 * @dev: hotplug device
 * @port_idx: port index (0-based)
 * @stage: protection stage (BUS_PROTECT_INIT, BUS_PROTECT_CLEANUP, etc.)
 */
static void tegra254_hp_rp_bus_protect(struct tegra254_hp_dev *dev, int port_idx,
				  int stage)
{
	switch (stage) {
	case BUS_PROTECT_INIT:
		{
			int ret;

			ret = tegra254_hp_map_mmio_resources(dev);
			if (ret) {
				dev_err(&dev->pdev->dev,
					"Failed to map MMIO resources during bus init: %d\n",
					ret);
				return;
			}
		}
		return;

	case BUS_PROTECT_CLEANUP:
		{
			int i;

			for (i = 0; i < HP_PORT_MAX; i++) {
				if (dev->mmio.mac_port_base[i])
					dev->mmio.mac_port_base[i] = NULL;
			}
			if (dev->mmio.top_base)
				dev->mmio.top_base = NULL;
			if (dev->mmio.protect_base)
				dev->mmio.protect_base = NULL;
			if (dev->mmio.ckm_base)
				dev->mmio.ckm_base = NULL;
		}
		return;

	case BUS_PROTECT_CABLE_REMOVAL:
	case BUS_PROTECT_CABLE_PLUGIN:
		{
			struct rp_bus_mmio_info *mmio_info =
			    &dev->pd->rp_bus_mmio;
			void __iomem *mac_base;

			if (port_idx >= dev->pd->port_nums)
				return;

			mac_base = dev->mmio.mac_port_base[port_idx];
			if (!mac_base)
				return;

			if (stage == BUS_PROTECT_CABLE_REMOVAL) {
				tegra254_hp_reg_update_bits(mac_base,
						       mmio_info->mac.init_ctrl,
						       mmio_info->mac.ltssm_bit,
						       false);
			tegra254_hp_reg_update_bits(mac_base,
					       mmio_info->mac.init_ctrl,
					       mmio_info->mac.phy_rst_bit,
					       false);
				return;
			}

		tegra254_hp_toggle_update_bit(dev->mmio.top_base,
					 mmio_info->top.ctrl,
					 mmio_info->top.port_bits[port_idx],
					 mmio_info->top.update_bit,
					 false);
			udelay(TEGRA254_HP_DELAY_SHORT_US);

			tegra254_hp_bus_protect_enable(dev, port_idx);
			usleep_range(TEGRA254_HP_DELAY_BUS_PROTECT_US,
				     TEGRA254_HP_DELAY_BUS_PROTECT_US + 1000);

			tegra254_hp_reg_update_bits(mac_base,
					       mmio_info->mac.init_ctrl,
					       mmio_info->mac.phy_rst_bit,
					       true);
			tegra254_hp_reg_update_bits(mac_base,
					       mmio_info->mac.init_ctrl,
					       mmio_info->mac.ltssm_bit, true);
			usleep_range(TEGRA254_HP_DELAY_PHY_RESET_US,
				     TEGRA254_HP_DELAY_PHY_RESET_US + 1000);

			tegra254_hp_bus_protect_disable(dev, port_idx);

		tegra254_hp_toggle_update_bit(dev->mmio.top_base,
					 mmio_info->top.ctrl,
					 mmio_info->top.port_bits[port_idx],
					 mmio_info->top.update_bit,
					 true);
		}
		break;

	default:
		dev_warn(&dev->pdev->dev, "Unknown bus protect stage: %d\n",
			 stage);
		break;
	}
}

static void remove_device(struct tegra254_hp_dev *dev);
static int rescan_device(struct tegra254_hp_dev *dev);
static int tegra254_hp_enable_slot(struct hotplug_slot *slot);
static int tegra254_hp_disable_slot(struct hotplug_slot *slot);
static int tegra254_hp_get_adapter_status(struct hotplug_slot *slot, u8 *value);
static void tegra254_hp_disable_slot_work(struct work_struct *work);
static void tegra254_hp_probe_hp_work(struct work_struct *work);

static bool tegra254_hp_is_enabled(struct tegra254_hp_dev *hp_dev)
{
	unsigned long flags;
	bool enabled;

	spin_lock_irqsave(&hp_dev->lock, flags);
	enabled = hp_dev->hotplug_enabled;
	spin_unlock_irqrestore(&hp_dev->lock, flags);
	return enabled;
}

/**
 * tegra254_hp_power_on_start - Arm BOOT SM and assert EN
 * @hp_dev: hotplug device
 *
 * Caller must hold slot_mutex. Shared by cable plug, probe sync, and
 * ensure_powered (slot-power) so there is one power-on sequence.
 */
static void tegra254_hp_power_on_start(struct tegra254_hp_dev *hp_dev)
{
	hp_dev->boot_state = BOOT_POWERING_ON;
	hp_dev->last_boot_val =
		gpiod_get_value(hp_dev->pins[PCIE_PIN_BOOT].desc);
	gpiod_set_value(hp_dev->pins[PCIE_PIN_EN].desc, 1);
}

/**
 * tegra254_hp_disable_all_slots - disable_slot for every registered port
 * @hp_dev: hotplug device
 *
 * Must not be called while holding slot_mutex (slot ops take it).
 */
static void tegra254_hp_disable_all_slots(struct tegra254_hp_dev *hp_dev)
{
	int i;

	for (i = 0; i < hp_dev->pd->port_nums; i++) {
		if (hp_dev->slots[i].root_port)
			hp_dev->slots[i].slot.ops->disable_slot(
				&hp_dev->slots[i].slot);
	}
}

static const struct hotplug_slot_ops tegra254_hp_slot_ops = {
	.enable_slot		= tegra254_hp_enable_slot,
	.disable_slot		= tegra254_hp_disable_slot,
	.get_adapter_status	= tegra254_hp_get_adapter_status,
};

static int tegra254_hp_get_adapter_status(struct hotplug_slot *hotplug_slot, u8 *value)
{
	struct tegra254_hp_slot *slot = container_of(hotplug_slot, struct tegra254_hp_slot, slot);
	struct tegra254_hp_dev *hp_dev = slot->hp_dev;

	/* PRSNT low = cable inserted = adapter present */
	*value = gpiod_get_value(hp_dev->pins[PCIE_PIN_PRSNT].desc) ? 0 : 1;
	return 0;
}

static int tegra254_hp_enable_slot(struct hotplug_slot *hotplug_slot)
{
	struct tegra254_hp_slot *slot = container_of(hotplug_slot, struct tegra254_hp_slot, slot);
	struct tegra254_hp_dev *hp_dev = slot->hp_dev;
	struct pci_bus *bus;
	int err = 0;

	if (!tegra254_hp_is_enabled(hp_dev))
		return -EPERM;

	mutex_lock(&hp_dev->slot_mutex);
	if (hp_dev->slots_enabled_count == 0) {
		dev_dbg(&hp_dev->pdev->dev, "slot API: first enable_slot, rescan_device\n");
		err = rescan_device(hp_dev);
		if (err) {
			mutex_unlock(&hp_dev->slot_mutex);
			return err;
		}
	}
	hp_dev->slots_enabled_count++;
	mutex_unlock(&hp_dev->slot_mutex);

	bus = slot->root_port->subordinate;
	if (bus) {
		dev_dbg(&hp_dev->pdev->dev, "slot API: enable_slot port %d, pci_rescan_bus\n",
			 slot->port_idx);
		pci_lock_rescan_remove();
		pci_rescan_bus(bus);
		pci_unlock_rescan_remove();
	}

	return 0;
}

/**
 * tegra254_hp_disable_slot_work - Remove devices on one port, power down if last
 * @work: work struct
 *
 * Deferred from disable_slot so endpoint .remove is not nested in the
 * hotplug callback.
 */
static void tegra254_hp_disable_slot_work(struct work_struct *work)
{
	struct tegra254_hp_disable_work *dwork =
		container_of(work, struct tegra254_hp_disable_work, work);
	struct tegra254_hp_dev *hp_dev = dwork->hp_dev;
	int port_idx = dwork->port_idx;
	struct tegra254_hp_slot *slot = &hp_dev->slots[port_idx];
	struct pci_bus *bus;
	struct pci_dev *dev, *tmp;

	if (!slot->root_port)
		return;

	dev_dbg(&hp_dev->pdev->dev, "slot API: disable_slot_work start port %d\n",
		port_idx);

	bus = slot->root_port->subordinate;
	if (bus) {
		dev_dbg(&hp_dev->pdev->dev,
			"slot API: disable_slot_work port %d release drivers\n",
			port_idx);
		list_for_each_entry_safe(dev, tmp, &bus->devices, bus_list)
			device_release_driver(&dev->dev);
		/*
		 * Hold pci_rescan_remove_lock and call the unlocked helper.
		 * Do NOT call pci_stop_and_remove_bus_device_locked() here:
		 * that helper takes the same lock again and self-deadlocks.
		 * (pciehp does the same: lock + pci_stop_and_remove_bus_device.)
		 */
		dev_dbg(&hp_dev->pdev->dev,
			"slot API: disable_slot_work port %d before pci_lock_rescan_remove\n",
			port_idx);
		pci_lock_rescan_remove();
		dev_dbg(&hp_dev->pdev->dev,
			"slot API: disable_slot_work port %d remove children\n",
			port_idx);
		list_for_each_entry_safe_reverse(dev, tmp, &bus->devices, bus_list)
			pci_stop_and_remove_bus_device(dev);
		pci_unlock_rescan_remove();
		dev_dbg(&hp_dev->pdev->dev,
			"slot API: disable_slot_work port %d after pci_unlock_rescan_remove\n",
			port_idx);
	}

	dev_dbg(&hp_dev->pdev->dev,
		"slot API: disable_slot_work port %d before slot_mutex (enabled_count=%d)\n",
		port_idx, hp_dev->slots_enabled_count);
	mutex_lock(&hp_dev->slot_mutex);
	if (hp_dev->slots_enabled_count > 0) {
		hp_dev->slots_enabled_count--;
		if (hp_dev->slots_enabled_count == 0) {
			dev_dbg(&hp_dev->pdev->dev,
				"slot API: disable_slot_work port %d calling remove_device\n",
				port_idx);
			remove_device(hp_dev);
		}
	}
	mutex_unlock(&hp_dev->slot_mutex);
	dev_dbg(&hp_dev->pdev->dev, "slot API: disable_slot_work done port %d\n",
		port_idx);
}

static int tegra254_hp_disable_slot(struct hotplug_slot *hotplug_slot)
{
	struct tegra254_hp_slot *slot = container_of(hotplug_slot, struct tegra254_hp_slot, slot);
	struct tegra254_hp_dev *hp_dev = slot->hp_dev;

	if (!tegra254_hp_is_enabled(hp_dev))
		return -EPERM;

	dev_dbg(&hp_dev->pdev->dev,
		"slot API: disable_slot port %d schedule disable_slot_work\n",
		slot->port_idx);
	schedule_work(&hp_dev->disable_work[slot->port_idx].work);
	dev_dbg(&hp_dev->pdev->dev,
		"slot API: disable_slot port %d returned after schedule_work\n",
		slot->port_idx);
	return 0;
}

/**
 * tegra254_hp_prsnt_work - Work for PRSNT pin: cable in/out and slot enable/disable
 * @work: work struct
 */
static void tegra254_hp_prsnt_work(struct work_struct *work)
{
	struct tegra254_hp_dev *hp_dev = container_of(work, struct tegra254_hp_dev, prsnt_work);
	int prsnt_val;

	if (!tegra254_hp_is_enabled(hp_dev))
		return;

	prsnt_val = gpiod_get_value(hp_dev->pins[PCIE_PIN_PRSNT].desc);
	/* PRSNT low = cable inserted */
	mutex_lock(&hp_dev->slot_mutex);
	if (prsnt_val) {
		dev_dbg(&hp_dev->pdev->dev, "prsnt_work: cable removed, disable_slot for all ports\n");
		hp_dev->cable_present = false;
		hp_dev->boot_state = BOOT_IDLE;
		mutex_unlock(&hp_dev->slot_mutex);
		tegra254_hp_disable_all_slots(hp_dev);
	} else {
		dev_dbg(&hp_dev->pdev->dev, "prsnt_work: cable inserted, EN=1 boot_state=BOOT_POWERING_ON\n");
		hp_dev->cable_present = true;
		tegra254_hp_power_on_start(hp_dev);
		mutex_unlock(&hp_dev->slot_mutex);
	}
}

/**
 * tegra254_hp_boot_work - Work for BOOT pin: state machine, enable_slot when ready
 * @work: work struct
 */
static void tegra254_hp_boot_work(struct work_struct *work)
{
	struct tegra254_hp_dev *hp_dev = container_of(work, struct tegra254_hp_dev, boot_work);
	int boot_val;
	int i;
	bool do_enable = false;

	if (!tegra254_hp_is_enabled(hp_dev))
		return;

	mutex_lock(&hp_dev->slot_mutex);
	boot_val = gpiod_get_value(hp_dev->pins[PCIE_PIN_BOOT].desc);

	switch (hp_dev->boot_state) {
	case BOOT_IDLE:
		break;
	case BOOT_POWERING_ON:
		if (boot_val) {
			dev_dbg(&hp_dev->pdev->dev, "boot_work: BOOT_POWERING_ON -> BOOT_FW_LOADING\n");
			hp_dev->boot_state = BOOT_FW_LOADING;
		}
		break;
	case BOOT_FW_LOADING:
		/* BOOT low then high = firmware ready */
		if (hp_dev->last_boot_val == 0 && boot_val) {
			dev_dbg(&hp_dev->pdev->dev, "boot_work: BOOT_READY\n");
			hp_dev->boot_state = BOOT_READY;
			/*
			 * Physical plug: nobody is blocked in enable_slot yet —
			 * call enable_slot. Slot-power / ensure path: a waiter
			 * holds the bring-up; only signal completion.
			 */
			do_enable = !hp_dev->boot_irq_waiter;
			complete_all(&hp_dev->boot_fw_ready);
		}
		break;
	case BOOT_READY:
		break;
	}
	hp_dev->last_boot_val = boot_val;
	mutex_unlock(&hp_dev->slot_mutex);

	/*
	 * enable_slot takes slot_mutex — must not call it while holding
	 * slot_mutex (non-recursive → self-deadlock).
	 */
	if (do_enable) {
		dev_dbg(&hp_dev->pdev->dev, "boot_work: enable_slot for all ports\n");
		for (i = 0; i < hp_dev->pd->port_nums; i++) {
			if (hp_dev->slots[i].root_port)
				hp_dev->slots[i].slot.ops->enable_slot(&hp_dev->slots[i].slot);
		}
	}
}

/**
 * get_port_root_port - Get PCI root port device for a port
 * @hp_dev: hotplug device
 * @port_idx: port index
 *
 * Returns cached or newly found root port, or NULL if not found.
 */
static struct pci_dev *get_port_root_port(struct tegra254_hp_dev *hp_dev,
					  int port_idx)
{
	struct pcie_port_info *port;

	if (!hp_dev->pd || port_idx >= hp_dev->pd->port_nums)
		return NULL;

	port = &hp_dev->pd->ports[port_idx];

	if (!hp_dev->slots[port_idx].root_port) {
		hp_dev->slots[port_idx].root_port =
		    pci_get_domain_bus_and_slot(port->domain,
						port->bus, port->devfn);
		if (!hp_dev->slots[port_idx].root_port) {
			dev_warn(&hp_dev->pdev->dev,
				 "Root port not found for domain %d bus %d\n",
				 port->domain, port->bus);
			return NULL;
		}
	}

	return hp_dev->slots[port_idx].root_port;
}

/**
 * remove_device - Remove PCIe devices and power down hardware
 * @dev: hotplug device
 */
static void remove_device(struct tegra254_hp_dev *dev)
{
	int i;

	dev_dbg(&dev->pdev->dev, "Cable removal\n");

	for (i = 0; i < dev->pd->port_nums; i++)
		tegra254_hp_rp_bus_protect(dev, i, BUS_PROTECT_CABLE_REMOVAL);

	gpiod_set_value(dev->pins[PCIE_PIN_PERST].desc, 0);
	tegra254_hp_change_pinctrl_state(dev, "default");
	tegra254_hp_ckm_control(dev, true);
	gpiod_set_value(dev->pins[PCIE_PIN_EN].desc, 0);
	dev->boot_state = BOOT_IDLE;
	complete_all(&dev->boot_fw_ready);
}

/**
 * polling_link_to_l0 - Poll until all PCIe ports reach L0 state
 * @dev: hotplug device
 *
 * Returns: 0 on success, negative error code on failure
 */
static int polling_link_to_l0(struct tegra254_hp_dev *dev)
{
	struct pci_dev *pci_dev;
	u32 ltssm_vals[HP_PORT_MAX] = { 0 };
	int count = 0;
	int i;
	bool all_l0;

	if (!dev->pd->ltssm_reg || !dev->pd->ltssm_l0_state)
		return 0;	/* Skip if not configured */

	/* Poll until all ports reach L0 state */
	all_l0 = false;
	while (!all_l0) {
		all_l0 = true;

		for (i = 0; i < dev->pd->port_nums; i++) {
			pci_dev = get_port_root_port(dev, i);
			if (!pci_dev) {
				all_l0 = false;
				continue;
			}

			pci_read_config_dword(pci_dev, dev->pd->ltssm_reg,
					      &ltssm_vals[i]);
			if ((ltssm_vals[i] & dev->pd->ltssm_l0_state) != dev->pd->ltssm_l0_state)
				all_l0 = false;
		}

		if (all_l0)
			break;

		usleep_range(TEGRA254_HP_POLL_SLEEP_US, TEGRA254_HP_POLL_SLEEP_US + 1000);
		count++;

		if (count > HP_POLL_CNT_MAX) {
			dev_err(&dev->pdev->dev,
				"Timeout waiting for link to reach L0 (reached max count)\n");
			break;
		}
	}

	if (count > HP_POLL_CNT_MAX)
		return -ETIMEDOUT;

	return 0;
}

/**
 * tegra254_hp_ensure_powered_and_fw_ready - EN=1 and wait for BOOT ready via IRQ
 * @dev: hotplug device
 *
 * Waits for boot_work (BOOT GPIO IRQs) to reach BOOT_READY. Used when
 * enable_slot runs before FW is ready (e.g. slot power after remove_device).
 *
 * Caller must hold slot_mutex on entry; it may be dropped while waiting.
 *
 * Returns: 0 on success, -ETIMEDOUT if BOOT never becomes ready
 */
static int tegra254_hp_ensure_powered_and_fw_ready(struct tegra254_hp_dev *dev)
{
	unsigned long timeout;
	int ret = 0;

	if (dev->boot_state == BOOT_READY &&
	    gpiod_get_value(dev->pins[PCIE_PIN_EN].desc))
		return 0;

	dev_dbg(&dev->pdev->dev,
		"%s: EN=1, wait for BOOT IRQ ready (was boot_state=%d)\n",
		__func__, dev->boot_state);

	reinit_completion(&dev->boot_fw_ready);
	dev->boot_irq_waiter = true;
	tegra254_hp_power_on_start(dev);
	mutex_unlock(&dev->slot_mutex);

	/* Kick in case BOOT level already matches without a new edge */
	schedule_work(&dev->boot_work);

	timeout = wait_for_completion_timeout(
		&dev->boot_fw_ready,
		msecs_to_jiffies(TEGRA254_HP_BOOT_POLL_MAX *
				 (TEGRA254_HP_POLL_SLEEP_US / 1000)));

	mutex_lock(&dev->slot_mutex);
	dev->boot_irq_waiter = false;

	if (!timeout || dev->boot_state != BOOT_READY) {
		dev_err(&dev->pdev->dev,
			"Timeout waiting for CX7 BOOT ready (IRQ path)\n");
		dev->boot_state = BOOT_IDLE;
		ret = -ETIMEDOUT;
	} else {
		dev_dbg(&dev->pdev->dev, "%s: BOOT ready via boot_work\n", __func__);
	}

	return ret;
}

/**
 * rescan_device - Rescan PCIe bus to discover devices
 * @dev: hotplug device
 *
 * Returns: 0 on success, negative error code on failure
 */
static int rescan_device(struct tegra254_hp_dev *dev)
{
	struct pci_dev *pci_dev;
	int i, err;
	int put_until = 0;

	dev_dbg(&dev->pdev->dev,
		"%s: start (pinctrl, CKM, PERST, bus protect, L0, retrain)\n",
		__func__);
	err = tegra254_hp_ensure_powered_and_fw_ready(dev);
	if (err)
		return err;

	err = tegra254_hp_change_pinctrl_state(dev, "clkreqn");
	if (err)
		return err;

	tegra254_hp_ckm_control(dev, false);
	usleep_range(TEGRA254_HP_DELAY_STANDARD_US, TEGRA254_HP_DELAY_STANDARD_US + 1000);

	for (i = 0; i < dev->pd->port_nums; i++) {
		pci_dev = get_port_root_port(dev, i);
		if (!pci_dev)
			continue;

		err = pm_runtime_resume_and_get(&pci_dev->dev);
		if (err < 0) {
			dev_err(&dev->pdev->dev,
				"Runtime resume failed for %s: %d\n",
				pci_name(pci_dev), err);
			put_until = i;
			goto out;
		}
		put_until = i + 1;
	}

	gpiod_set_value(dev->pins[PCIE_PIN_PERST].desc, 1);

	for (i = 0; i < dev->pd->port_nums; i++)
		tegra254_hp_rp_bus_protect(dev, i, BUS_PROTECT_CABLE_PLUGIN);

	err = polling_link_to_l0(dev);
	if (err)
		goto out;

	for (i = 0; i < dev->pd->port_nums; i++) {
		pci_dev = get_port_root_port(dev, i);
		if (!pci_dev)
			continue;

		/* After cable reinsertion the link can stay at Gen1; retrain to max speed. */
		err = pcie_retrain_link(pci_dev, false);
		if (err)
			dev_err(&dev->pdev->dev,
				"pcie_retrain_link failed for %s: %d\n",
				pci_name(pci_dev), err);
	}

	msleep(TEGRA254_HP_DELAY_LINK_STABLE_MS);
	err = 0;
out:
	for (i = 0; i < put_until; i++) {
		pci_dev = get_port_root_port(dev, i);
		if (pci_dev)
			pm_runtime_put(&pci_dev->dev);
	}
	return err;
}

/**
 * tegra254_hp_work - Threaded IRQ handler: schedule PRSNT or BOOT work
 * @irq: interrupt number
 * @dev_id: GPIO context pointer
 *
 * PRSNT and BOOT pins set pending_prsnt or pending_boot in the hardirq;
 * this thread schedules the corresponding work and returns.
 */
static irqreturn_t tegra254_hp_work(int irq, void *dev_id)
{
	struct tegra254_hp_gpio_ctx *app_ctx = dev_id;
	struct tegra254_hp_dev *hp_dev;
	unsigned long flags;

	if (!app_ctx || !app_ctx->hp_dev)
		return IRQ_NONE;

	hp_dev = app_ctx->hp_dev;

	spin_lock_irqsave(&hp_dev->lock, flags);

	if (hp_dev->pending_prsnt) {
		hp_dev->pending_prsnt = false;
		spin_unlock_irqrestore(&hp_dev->lock, flags);
		dev_dbg(&hp_dev->pdev->dev, "IRQ thread: schedule prsnt_work\n");
		schedule_work(&hp_dev->prsnt_work);
		return IRQ_HANDLED;
	}
	if (hp_dev->pending_boot) {
		hp_dev->pending_boot = false;
		spin_unlock_irqrestore(&hp_dev->lock, flags);
		dev_dbg(&hp_dev->pdev->dev, "IRQ thread: schedule boot_work\n");
		schedule_work(&hp_dev->boot_work);
		return IRQ_HANDLED;
	}

	spin_unlock_irqrestore(&hp_dev->lock, flags);
	return IRQ_HANDLED;
}

/**
 * hotplug_irq_handler - GPIO interrupt handler for hotplug events
 * @irq: interrupt number
 * @dev_id: GPIO context pointer
 *
 * Handles presence detection and boot status GPIO interrupts.
 */
static irqreturn_t hotplug_irq_handler(int irq, void *dev_id)
{
	struct tegra254_hp_gpio_ctx *app_ctx = dev_id;
	struct tegra254_hp_dev *hp_dev = app_ctx->hp_dev;
	struct gpio_acpi_context *gpio_ctx = app_ctx->ctx;
	unsigned long flags;

	spin_lock_irqsave(&hp_dev->lock, flags);

	if (gpio_ctx->pin == hp_dev->prsnt_pin) {
		/* Defer to threaded handler (cannot sleep in hardirq) */
		hp_dev->pending_prsnt = true;
		spin_unlock_irqrestore(&hp_dev->lock, flags);
		return IRQ_WAKE_THREAD;
	}
	if (gpio_ctx->pin == hp_dev->boot_pin) {
		hp_dev->pending_boot = true;
		spin_unlock_irqrestore(&hp_dev->lock, flags);
		return IRQ_WAKE_THREAD;
	}

	spin_unlock_irqrestore(&hp_dev->lock, flags);
	dev_err(gpio_ctx->dev,
		"Unknown GPIO pin event: pin=%d irq=%d value=%d\n",
		gpio_ctx->pin, irq, gpiod_get_value(app_ctx->desc));
	return IRQ_HANDLED;
}

/**
 * acpi_gpio_collect_handler - ACPI resource handler to collect all GPIO resources
 * @ares: ACPI resource structure
 * @context: Pointer to acpi_gpio_walk_context
 *
 * Returns: AE_OK to continue iteration
 */
static acpi_status acpi_gpio_collect_handler(struct acpi_resource *ares,
					     void *context)
{
	struct acpi_gpio_walk_context *walk_ctx = context;
	struct acpi_resource_gpio *agpio;
	int length;

	if (ares->type != ACPI_RESOURCE_TYPE_GPIO)
		return AE_OK;

	if (walk_ctx->count >= PCIE_PIN_MAX) {
		dev_warn(walk_ctx->dev,
			 "Too many GPIO resources, truncating at %d\n",
			 PCIE_PIN_MAX);
		return AE_OK;
	}

	agpio = &ares->data.gpio;

	if (!agpio->pin_table || agpio->pin_table_length == 0) {
		dev_warn(walk_ctx->dev, "GPIO resource has no pin table\n");
		return AE_OK;
	}

	walk_ctx->gpios[walk_ctx->count].pin = agpio->pin_table[0];

	if (agpio->resource_source.string_ptr) {
		length = min_t(int, agpio->resource_source.string_length, 15);
		memcpy(walk_ctx->gpios[walk_ctx->count].resource_source,
		       agpio->resource_source.string_ptr, length);
		walk_ctx->gpios[walk_ctx->count].resource_source[length] = '\0';
	} else {
		walk_ctx->gpios[walk_ctx->count].resource_source[0] = '\0';
	}
	walk_ctx->count++;
	return AE_OK;
}

/**
 * tegra254_hp_walk_acpi_gpios - Walk ACPI _CRS to collect all GPIO resources
 * @pdev: Platform device
 * @walk_ctx: Context structure to fill with GPIO information
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_walk_acpi_gpios(struct platform_device *pdev,
				  struct acpi_gpio_walk_context *walk_ctx)
{
	struct acpi_device *adev;
	acpi_status status;

	adev = ACPI_COMPANION(&pdev->dev);
	if (!adev) {
		dev_err(&pdev->dev, "Failed to get ACPI companion device\n");
		return -ENODEV;
	}

	memset(walk_ctx, 0, sizeof(*walk_ctx));
	walk_ctx->dev = &pdev->dev;

	status = acpi_walk_resources(adev->handle, METHOD_NAME__CRS,
				     acpi_gpio_collect_handler, walk_ctx);
	if (ACPI_FAILURE(status)) {
		dev_err(&pdev->dev, "Failed to walk ACPI GPIO resources: %s\n",
			acpi_format_exception(status));
		return -EIO;
	}

	dev_dbg(&pdev->dev, "Found %d GPIO resources via ACPI walk\n",
		walk_ctx->count);

	if (walk_ctx->count == 0) {
		dev_err(&pdev->dev, "No GPIO resources found in ACPI _CRS\n");
		return -ENODEV;
	}

	return 0;
}

/**
 * acpi_gpio_lookup_handler - ACPI resource handler to look up a specific GPIO pin
 * @ares: ACPI resource being processed
 * @context: Pointer to acpi_gpio_parse_context
 *
 * Returns: AE_OK to continue iteration
 */
static acpi_status acpi_gpio_lookup_handler(struct acpi_resource *ares,
					    void *context)
{
	struct acpi_gpio_parse_context *parse_ctx = context;
	struct gpio_acpi_context *ctx = parse_ctx->ctx;
	struct tegra254_hp_dev *hp_dev = parse_ctx->hp_dev;
	struct acpi_resource_gpio *agpio;
	char vendor[MAX_VENDOR_DATA_LEN + 1];
	int length;

	if (ares->type != ACPI_RESOURCE_TYPE_GPIO)
		return AE_OK;

	agpio = &ares->data.gpio;

	if (ctx->pin != agpio->pin_table[0])
		return AE_OK;

	ctx->valid = 1;
	ctx->debounce_timeout_us = agpio->debounce_timeout * 10;
	ctx->wake_capable = agpio->wake_capable;
	ctx->connection_type = agpio->connection_type;

	if (agpio->vendor_length && agpio->vendor_data && hp_dev) {
		length = min_t(int, agpio->vendor_length, MAX_VENDOR_DATA_LEN);
		memcpy(vendor, agpio->vendor_data, length);
		vendor[length] = '\0';

		if (!strncmp(vendor, "BOOT", 4))
			hp_dev->boot_pin = ctx->pin;
		else if (!strncmp(vendor, "PRSNT", 5))
			hp_dev->prsnt_pin = ctx->pin;
	}

	if (agpio->triggering == ACPI_EDGE_SENSITIVE) {
		if (agpio->polarity == ACPI_ACTIVE_LOW)
			ctx->irq_flags = IRQF_TRIGGER_FALLING;
		else if (agpio->polarity == ACPI_ACTIVE_HIGH)
			ctx->irq_flags = IRQF_TRIGGER_RISING;
		else
			ctx->irq_flags =
			    (IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING);
	} else {
		if (agpio->polarity == ACPI_ACTIVE_LOW)
			ctx->irq_flags = IRQF_TRIGGER_LOW;
		else
			ctx->irq_flags = IRQF_TRIGGER_HIGH;
	}

	return AE_OK;
}

/**
 * pci_devices_present_on_domain() - Check if PCI devices exist on a domain
 * @domain: PCI domain number to check
 *
 * Returns: true if any PCI devices are present on the specified domain,
 * false otherwise. This is used as a safety check before hardware shutdown.
 */
static bool pci_devices_present_on_domain(int domain)
{
	struct pci_bus *bus;
	struct pci_dev *dev;
	bool has_endpoint_devices = false;

	bus = pci_find_bus(domain, 1);
	if (!bus)
		return false;

	list_for_each_entry(dev, &bus->devices, bus_list) {
		has_endpoint_devices = true;
		break;
	}

	return has_endpoint_devices;
}

static bool tegra254_hp_any_pci_devices(struct tegra254_hp_dev *hp_dev)
{
	int i;

	for (i = 0; i < hp_dev->pd->port_nums; i++) {
		if (pci_devices_present_on_domain(hp_dev->pd->ports[i].domain))
			return true;
	}
	return false;
}

static int tegra254_hp_registered_slot_count(struct tegra254_hp_dev *hp_dev)
{
	int i, n = 0;

	for (i = 0; i < hp_dev->pd->port_nums; i++) {
		if (hp_dev->slots[i].root_port)
			n++;
	}
	return n;
}

/**
 * tegra254_hp_probe_hp_work - Sync PCI/HW to cable after hotplug is enabled
 * @work: work struct
 *
 * Scheduled on hotplug_enabled 0→1 (not at probe). Tears down if no cable
 * but CX7 is present, seeds slot state if cable+devices, or powers on if
 * cable and empty.
 */
static void tegra254_hp_probe_hp_work(struct work_struct *work)
{
	struct tegra254_hp_dev *hp_dev = container_of(work, struct tegra254_hp_dev,
						 probe_hp_work);
	bool devices;
	int nslots;

	if (!tegra254_hp_is_enabled(hp_dev)) {
		dev_dbg(&hp_dev->pdev->dev,
			"probe_hp_work: hotplug disabled, skip cable sync\n");
		return;
	}

	hp_dev->cable_present =
		!gpiod_get_value(hp_dev->pins[PCIE_PIN_PRSNT].desc);
	devices = tegra254_hp_any_pci_devices(hp_dev);
	nslots = tegra254_hp_registered_slot_count(hp_dev);

	dev_dbg(&hp_dev->pdev->dev,
		"probe_hp_work: cable_present=%d devices=%d slots=%d\n",
		hp_dev->cable_present, devices, nslots);

	if (!hp_dev->cable_present) {
		if (!devices || !nslots)
			return;
		/*
		 * Seed enabled count so the last disable_slot_work calls
		 * remove_device (power down). Do not hold slot_mutex across
		 * disable_slot (it only schedules work today).
		 */
		mutex_lock(&hp_dev->slot_mutex);
		hp_dev->slots_enabled_count = nslots;
		hp_dev->boot_state = BOOT_IDLE;
		mutex_unlock(&hp_dev->slot_mutex);
		dev_dbg(&hp_dev->pdev->dev,
			"probe_hp_work: no cable, CX7 present — disable_slot\n");
		tegra254_hp_disable_all_slots(hp_dev);
		return;
	}

	if (devices) {
		/* FW already enumerated CX7 — sync software so later unplug works */
		mutex_lock(&hp_dev->slot_mutex);
		hp_dev->slots_enabled_count = nslots;
		hp_dev->boot_state = BOOT_READY;
		mutex_unlock(&hp_dev->slot_mutex);
		dev_dbg(&hp_dev->pdev->dev,
			"probe_hp_work: cable + devices — seeded enabled_count=%d\n",
			nslots);
		return;
	}

	/* Cable present, no endpoints — start the same path as PRSNT insert */
	mutex_lock(&hp_dev->slot_mutex);
	tegra254_hp_power_on_start(hp_dev);
	mutex_unlock(&hp_dev->slot_mutex);
	dev_dbg(&hp_dev->pdev->dev,
		"probe_hp_work: cable, no devices — EN=1 BOOT_POWERING_ON\n");
}

static ssize_t hotplug_enabled_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct tegra254_hp_dev *hp_dev = dev_get_drvdata(dev);

	if (!hp_dev)
		return -EINVAL;

	return scnprintf(buf, PAGE_SIZE, "%d\n", hp_dev->hotplug_enabled ? 1 : 0);
}

static ssize_t hotplug_enabled_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct tegra254_hp_dev *hp_dev = dev_get_drvdata(dev);
	unsigned long flags;
	unsigned long val;
	bool was_enabled, now_enabled;
	int err;

	if (!hp_dev)
		return -EINVAL;

	err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	spin_lock_irqsave(&hp_dev->lock, flags);
	was_enabled = hp_dev->hotplug_enabled;
	hp_dev->hotplug_enabled = (val != 0);
	now_enabled = hp_dev->hotplug_enabled;
	spin_unlock_irqrestore(&hp_dev->lock, flags);

	dev_dbg(dev, "Hotplug %s\n", now_enabled ? "enabled" : "disabled");

	/*
	 * Enabling: run cable sync (boot tear-down / power-on). Disabling:
	 * stop reacting to new events; do not reverse current PCI/HW state.
	 */
	if (now_enabled && !was_enabled)
		schedule_work(&hp_dev->probe_hp_work);

	return count;
}

DEVICE_ATTR_RW(hotplug_enabled);

static struct attribute *tegra254_hp_attrs[] = {
	&dev_attr_hotplug_enabled.attr,
	NULL
};

static const struct attribute_group tegra254_hp_attr_group = {
	.name = "pcie_hotplug",
	.attrs = tegra254_hp_attrs
};

/**
 * gpio_acpi_setup - Setup GPIO ACPI context from _CRS
 * @pdev: platform device
 * @desc: GPIO descriptor
 * @hp_dev: hotplug device
 * @gpio_index: GPIO index
 *
 * Returns: GPIO ACPI context on success, NULL on failure
 */
static struct gpio_acpi_context *gpio_acpi_setup(struct platform_device *pdev,
						 struct gpio_desc *desc,
						 struct tegra254_hp_dev *hp_dev,
						 int gpio_index)
{
	struct acpi_gpio_parse_context parse_ctx;
	struct gpio_acpi_context *ctx;
	struct acpi_device *adev;
	acpi_status status;

	adev = ACPI_COMPANION(&pdev->dev);
	if (!adev) {
		dev_err(&pdev->dev, "Failed to get ACPI companion device\n");
		return NULL;
	}

	ctx = devm_kzalloc(&pdev->dev, sizeof(*ctx), GFP_KERNEL);
	if (!ctx)
		return NULL;

	ctx->pin =
	    desc_to_gpio(desc) -
	    gpio_device_get_base(gpiod_to_gpio_device(desc));
	ctx->dev = &pdev->dev;

	parse_ctx.ctx = ctx;
	parse_ctx.hp_dev = hp_dev;

	status = acpi_walk_resources(adev->handle, METHOD_NAME__CRS,
				     acpi_gpio_lookup_handler, &parse_ctx);
	if (ACPI_FAILURE(status)) {
		devm_kfree(&pdev->dev, ctx);
		return NULL;
	}

	if (ctx->valid) {
		if (gpio_index == PCIE_PIN_BOOT && hp_dev->boot_pin == -1) {
			hp_dev->boot_pin = ctx->pin;
		} else if (gpio_index == PCIE_PIN_PRSNT
			   && hp_dev->prsnt_pin == -1) {
			hp_dev->prsnt_pin = ctx->pin;
		}
		return ctx;
	}

	devm_kfree(&pdev->dev, ctx);
	return NULL;
}

/**
 * tegra254_hp_setup_irq - Setup IRQ for GPIO
 * @app_ctx: GPIO context
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_setup_irq(struct tegra254_hp_gpio_ctx *app_ctx)
{
	struct gpio_acpi_context *ctx = app_ctx->ctx;
	int irq, ret;

	irq = gpiod_to_irq(app_ctx->desc);
	if (irq < 0) {
		dev_err(ctx->dev, "Failed to get IRQ for GPIO\n");
		return irq;
	}

	if (ctx->wake_capable)
		enable_irq_wake(irq);

	ret = devm_request_threaded_irq(ctx->dev, irq,
					hotplug_irq_handler, tegra254_hp_work,
					ctx->irq_flags | IRQF_ONESHOT,
					"pcie_hotplug", app_ctx);
	if (ret)
		dev_err(ctx->dev, "Failed to request IRQ %d: %d\n", irq, ret);

	return ret;
}

/**
 * tegra254_hp_put_gpio_device - Release GPIO device reference
 * @data: GPIO device pointer
 */
static void tegra254_hp_put_gpio_device(void *data)
{
	struct gpio_device *gdev = data;

	gpio_device_put(gdev);
}

/**
 * tegra254_hp_count_dev_on_bus - Callback for pci_walk_bus to count matching devices
 * @pci_dev: device being visited
 * @data: pointer to struct tegra254_hp_discover_pcie_ctx
 *
 * Returns: 1 to stop walk early, 0 otherwise
 */
static int tegra254_hp_count_dev_on_bus(struct pci_dev *pci_dev, void *data)
{
	struct tegra254_hp_discover_pcie_ctx *ctx = data;

	if (pci_dev->vendor != ctx->pd->vendor_id || pci_dev->device != ctx->pd->device_id)
		return 0;

	if (!pci_dev->state_saved) {
		*ctx->retry = true;
		return 1;
	}

	(*ctx->count)++;
	return 0;
}

/**
 * tegra254_hp_discover_pcie_devices - Discover existing PCI devices on managed ports
 * @pdev: platform device
 * @pd: platform data
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_discover_pcie_devices(struct platform_device *pdev,
					struct tegra254_hp_plat_data *pd)
{
	int device_count = 0;
	bool retry = false;
	struct tegra254_hp_discover_pcie_ctx pcie_ctx = {
		.pd = pd, .count = &device_count, .retry = &retry
	};
	struct pci_dev *root_port = NULL;
	int i;

	if (!pd->vendor_id || !pd->device_id)
		return 0;

	for (i = 0; i < pd->port_nums; i++) {
		struct pcie_port_info *port = &pd->ports[i];

		root_port = pci_get_domain_bus_and_slot(port->domain,
							port->bus, port->devfn);
		if (!root_port) {
			dev_dbg(&pdev->dev,
				"Root port not yet available for domain %d bus %d\n",
				port->domain, port->bus);
			return -EPROBE_DEFER;
		}

		if (!root_port->subordinate) {
			pci_dev_put(root_port);
			dev_dbg(&pdev->dev,
				"Subordinate bus not yet enumerated for domain %d bus %d\n",
				port->domain, port->bus);
			return -EPROBE_DEFER;
		}

		pci_walk_bus(root_port->subordinate, tegra254_hp_count_dev_on_bus,
			     &pcie_ctx);
		pci_dev_put(root_port);

		if (retry)
			return -EPROBE_DEFER;
	}

	if (pd->num_devices && device_count != pd->num_devices) {
		dev_err(&pdev->dev,
			"Required number of devices not found. Expected=%d Actual=%d\n",
			pd->num_devices, device_count);
		return -ENODEV;
	}

	return 0;
}

/**
 * tegra254_hp_init_pcie_data - Initialize PCIe data from _DSD and discover devices
 * @pdev: platform device
 * @pd: platform data to populate
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_init_pcie_data(struct platform_device *pdev,
				 struct tegra254_hp_plat_data *pd)
{
	int ret;

	ret = tegra254_hp_parse_pcie_config_dsd(pdev, pd);
	if (ret) {
		dev_err(&pdev->dev,
			"Failed to parse PCIe configuration _DSD properties: %d\n",
			ret);
		return ret;
	}

	if (pd->port_nums == 0 || pd->port_nums >= HP_PORT_MAX) {
		dev_err(&pdev->dev,
			"Invalid port count from _DSD: %d (must be 1-%d)\n",
			pd->port_nums, HP_PORT_MAX - 1);
		return -EINVAL;
	}

	ret = tegra254_hp_discover_pcie_devices(pdev, pd);
	if (ret) {
		dev_dbg(&pdev->dev, "Device discovery failed: %d\n", ret);
		return ret;
	}

	return 0;
}

/**
 * tegra254_hp_enumerate_gpios - Enumerate GPIOs from ACPI
 * @pdev: Platform device
 * @hp_dev: Hotplug device structure
 *
 * Returns: Number of GPIOs found, or negative error code
 */
static int tegra254_hp_enumerate_gpios(struct platform_device *pdev,
				  struct tegra254_hp_dev *hp_dev)
{
	struct acpi_gpio_walk_context walk_ctx;
	struct fwnode_handle *gpio_fwnode = NULL;
	struct acpi_device *gpio_adev = NULL;
	acpi_handle gpio_handle;
	acpi_status status;
	int ret, i;

	ret = tegra254_hp_walk_acpi_gpios(pdev, &walk_ctx);
	if (ret) {
		dev_err(&pdev->dev, "Failed to walk ACPI GPIO resources: %d\n",
			ret);
		return ret;
	}

	if (walk_ctx.count < TEGRA254_HP_MIN_GPIO_COUNT) {
		dev_err(&pdev->dev,
			"Insufficient GPIOs from ACPI: required at least %d, got %d\n",
			TEGRA254_HP_MIN_GPIO_COUNT, walk_ctx.count);
		return -ENODEV;
	}

	if (walk_ctx.count == 0 || walk_ctx.gpios[0].resource_source[0] == '\0') {
		dev_err(&pdev->dev,
			"No resource_source in ACPI GPIO resources\n");
		return -ENODEV;
	}

	status = acpi_get_handle(NULL, walk_ctx.gpios[0].resource_source,
				 &gpio_handle);
	if (ACPI_FAILURE(status)) {
		dev_err(&pdev->dev,
			"Failed to get ACPI handle for GPIO controller %s\n",
			walk_ctx.gpios[0].resource_source);
		return -ENODEV;
	}

	gpio_adev = acpi_fetch_acpi_dev(gpio_handle);
	if (!gpio_adev) {
		dev_err(&pdev->dev,
			"Failed to get ACPI device for GPIO controller %s\n",
			walk_ctx.gpios[0].resource_source);
		return -ENODEV;
	}

	gpio_fwnode = acpi_fwnode_handle(gpio_adev);
	hp_dev->gdev = gpio_device_find_by_fwnode(gpio_fwnode);
	if (!hp_dev->gdev) {
		return dev_err_probe(&pdev->dev, -EPROBE_DEFER,
				     "GPIO controller not available\n");
	}

	ret = devm_add_action_or_reset(&pdev->dev, tegra254_hp_put_gpio_device,
				       hp_dev->gdev);
	if (ret) {
		gpio_device_put(hp_dev->gdev);
		hp_dev->gdev = NULL;
		dev_err(&pdev->dev, "Failed to register GPIO device cleanup\n");
		return ret;
	}

	hp_dev->gpio_count = walk_ctx.count;

	hp_dev->pins = devm_kzalloc(&pdev->dev,
				    sizeof(struct tegra254_hp_gpio_ctx) *
				    hp_dev->gpio_count, GFP_KERNEL);
	if (!hp_dev->pins) {
		dev_err(&pdev->dev, "Failed to allocate memory for GPIOs\n");
		return -ENOMEM;
	}

	for (i = 0; i < hp_dev->gpio_count; i++) {
		struct tegra254_hp_gpio_ctx *app_ctx = &hp_dev->pins[i];

		app_ctx->desc =
		    gpio_device_get_desc(hp_dev->gdev, walk_ctx.gpios[i].pin);
		if (IS_ERR(app_ctx->desc)) {
			dev_err(&pdev->dev,
				"Failed to get GPIO descriptor for ACPI pin %u (index %d): %ld\n",
				walk_ctx.gpios[i].pin, i,
				PTR_ERR(app_ctx->desc));
			return PTR_ERR(app_ctx->desc);
		}

		app_ctx->hp_dev = hp_dev;
	}

	return hp_dev->gpio_count;
}

/**
 * tegra254_hp_pci_notifier - PCI bus notifier to configure MPS for CX7 devices
 * @nb: notifier block
 * @action: bus notification action
 * @data: pointer to device being added/removed
 *
 * Returns: NOTIFY_OK on success, NOTIFY_DONE if not a CX7 device
 */
static int tegra254_hp_pci_notifier(struct notifier_block *nb, unsigned long action,
				void *data)
{
	struct device *dev = data;
	struct pci_dev *pdev = to_pci_dev(dev);
	struct tegra254_hp_dev *hp_dev;
	unsigned long flags;

	if (action != BUS_NOTIFY_ADD_DEVICE)
		return NOTIFY_DONE;

	hp_dev = container_of(nb, struct tegra254_hp_dev, pci_notifier);
	if (!hp_dev || !hp_dev->pd)
		return NOTIFY_DONE;

	spin_lock_irqsave(&hp_dev->lock, flags);
	if (!hp_dev->hotplug_enabled) {
		spin_unlock_irqrestore(&hp_dev->lock, flags);
		return NOTIFY_DONE;
	}
	spin_unlock_irqrestore(&hp_dev->lock, flags);

	if (!pdev || !hp_dev->pd->vendor_id || !hp_dev->pd->device_id)
		return NOTIFY_DONE;

	if (pdev->vendor != hp_dev->pd->vendor_id ||
	    pdev->device != hp_dev->pd->device_id)
		return NOTIFY_DONE;

	if (pdev->bus)
		pcie_bus_configure_settings(pdev->bus);

	return NOTIFY_OK;
}

/**
 * tegra254_hp_probe - Platform device probe function
 * @pdev: platform device
 *
 * Initializes the PCIe hotplug driver, parses ACPI resources, and sets up
 * GPIO interrupts and sysfs interface.
 *
 * Returns: 0 on success, negative error code on failure
 */
static int tegra254_hp_probe(struct platform_device *pdev)
{
	struct tegra254_hp_plat_data *pd;
	struct tegra254_hp_gpio_ctx *app_ctx;
	struct tegra254_hp_dev *hp_dev;
	int ret, i;

	pd = devm_kzalloc(&pdev->dev, sizeof(*pd), GFP_KERNEL);
	if (!pd)
		return -ENOMEM;

	ret = tegra254_hp_init_pcie_data(pdev, pd);
	if (ret)
		return ret;

	hp_dev = devm_kzalloc(&pdev->dev, sizeof(*hp_dev), GFP_KERNEL);
	if (!hp_dev)
		return -ENOMEM;

	hp_dev->pdev = pdev;
	hp_dev->pd = pd;
	hp_dev->boot_pin = -1;
	hp_dev->prsnt_pin = -1;
	hp_dev->hotplug_enabled = false;
	hp_dev->pending_prsnt = false;
	hp_dev->pending_boot = false;
	hp_dev->slots_enabled_count = 0;
	hp_dev->boot_state = BOOT_IDLE;
	hp_dev->last_boot_val = 0;
	hp_dev->boot_irq_waiter = false;
	init_completion(&hp_dev->boot_fw_ready);
	spin_lock_init(&hp_dev->lock);
	mutex_init(&hp_dev->slot_mutex);
	INIT_WORK(&hp_dev->prsnt_work, tegra254_hp_prsnt_work);
	INIT_WORK(&hp_dev->boot_work, tegra254_hp_boot_work);
	INIT_WORK(&hp_dev->probe_hp_work, tegra254_hp_probe_hp_work);
	for (i = 0; i < HP_PORT_MAX; i++) {
		hp_dev->disable_work[i].hp_dev = hp_dev;
		hp_dev->disable_work[i].port_idx = i;
		INIT_WORK(&hp_dev->disable_work[i].work, tegra254_hp_disable_slot_work);
	}

	ret = tegra254_hp_enumerate_gpios(pdev, hp_dev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to enumerate GPIOs from ACPI: %d\n",
			ret);
		return ret;
	}

	for (i = 0; i < hp_dev->gpio_count; i++) {
		app_ctx = &hp_dev->pins[i];

		app_ctx->ctx = gpio_acpi_setup(pdev, app_ctx->desc, hp_dev, i);
		if (!app_ctx->ctx) {
			dev_err(&pdev->dev, "Failed to setup GPIO %d\n", i);
			return -ENODEV;
		}

		gpiod_set_debounce(app_ctx->desc,
				   app_ctx->ctx->debounce_timeout_us);

		if (app_ctx->ctx->connection_type ==
		    ACPI_RESOURCE_GPIO_TYPE_INT) {
			ret = tegra254_hp_setup_irq(app_ctx);
			if (ret) {
				dev_err(&pdev->dev,
					"Failed to setup IRQ for GPIO %d\n", i);
				return ret;
			}
		}
	}

	hp_dev->cable_present = !gpiod_get_value(hp_dev->pins[PCIE_PIN_PRSNT].desc);
	hp_dev->last_boot_val = gpiod_get_value(hp_dev->pins[PCIE_PIN_BOOT].desc);

	platform_set_drvdata(pdev, hp_dev);

	ret = tegra254_hp_pinctrl_init(hp_dev);
	if (ret) {
		dev_err(&pdev->dev, "Pinmux init failed, ret: %d\n", ret);
		return ret;
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &tegra254_hp_attr_group);
	if (ret) {
		dev_err(&pdev->dev, "Sysfs creation failed: %d\n", ret);
		goto pinctrl_remove;
	}

	tegra254_hp_rp_bus_protect(hp_dev, 0, BUS_PROTECT_INIT);

	hp_dev->pci_notifier.notifier_call = tegra254_hp_pci_notifier;
	ret = bus_register_notifier(&pci_bus_type, &hp_dev->pci_notifier);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register PCI bus notifier: %d\n",
			ret);
		goto sysfs_remove;
	}

	for (i = 0; i < hp_dev->pd->port_nums; i++) {
		struct pci_dev *rp = get_port_root_port(hp_dev, i);
		char slot_name[24];

		if (!rp || !rp->subordinate)
			continue;
		hp_dev->slots[i].slot.ops = &tegra254_hp_slot_ops;
		hp_dev->slots[i].hp_dev = hp_dev;
		hp_dev->slots[i].port_idx = i;
		snprintf(slot_name, sizeof(slot_name), "tegra254-slot-%d", i);
		ret = pci_hp_register(&hp_dev->slots[i].slot, rp->subordinate,
				      0, slot_name);
		if (ret) {
			dev_err(&pdev->dev, "pci_hp_register slot %d failed: %d\n",
				i, ret);
			while (i--) {
				pci_hp_deregister(&hp_dev->slots[i].slot);
				pci_dev_put(hp_dev->slots[i].root_port);
				hp_dev->slots[i].root_port = NULL;
			}
			goto sysfs_remove;
		}
		dev_dbg(&pdev->dev, "slot API: registered slot %d (%s)\n", i, slot_name);
	}

	dev_dbg(&pdev->dev, "PCIe hotplug driver initialized successfully\n");
	/*
	 * Cable sync runs when userspace enables hotplug_enabled (0→1),
	 * not at probe — HP stays inert until then.
	 */
	return 0;

sysfs_remove:
	sysfs_remove_group(&pdev->dev.kobj, &tegra254_hp_attr_group);
pinctrl_remove:
	tegra254_hp_pinctrl_remove(hp_dev);
	return ret;
}

/**
 * tegra254_hp_remove - Platform device remove function
 * @pdev: platform device
 *
 * Cleans up GPIO pins, pinctrl, sysfs interface, and bus protection.
 */
static void tegra254_hp_remove(struct platform_device *pdev)
{
	struct tegra254_hp_dev *hp_dev = platform_get_drvdata(pdev);
	int i;

	if (!hp_dev)
		return;

	cancel_work_sync(&hp_dev->prsnt_work);
	cancel_work_sync(&hp_dev->boot_work);
	cancel_work_sync(&hp_dev->probe_hp_work);
	for (i = 0; i < HP_PORT_MAX; i++)
		cancel_work_sync(&hp_dev->disable_work[i].work);
	for (i = 0; i < hp_dev->pd->port_nums; i++) {
		if (hp_dev->slots[i].root_port) {
			pci_hp_deregister(&hp_dev->slots[i].slot);
			pci_dev_put(hp_dev->slots[i].root_port);
			hp_dev->slots[i].root_port = NULL;
		}
	}

	sysfs_remove_group(&pdev->dev.kobj, &tegra254_hp_attr_group);

	bus_unregister_notifier(&pci_bus_type, &hp_dev->pci_notifier);

	tegra254_hp_rp_bus_protect(hp_dev, 0, BUS_PROTECT_CLEANUP);

	tegra254_hp_pinctrl_remove(hp_dev);

	platform_set_drvdata(pdev, NULL);
}

static const struct acpi_device_id tegra254_hp_acpi_match[] = {
	{"MTKP0001", 0},
	{}
};

MODULE_DEVICE_TABLE(acpi, tegra254_hp_acpi_match);

static struct platform_driver tegra254_hp_driver = {
	.probe = tegra254_hp_probe,
	.remove = tegra254_hp_remove,
	.driver = {
		.name = "tegra254-pcie-hotplug",
		.acpi_match_table = ACPI_PTR(tegra254_hp_acpi_match),
	},
};

module_platform_driver(tegra254_hp_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Tegra254 PCIe power management for NVIDIA DGX Spark");
