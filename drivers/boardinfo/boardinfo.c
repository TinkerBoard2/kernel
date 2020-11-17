#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/boardinfo.h>

static int hw_id0, hw_id1, hw_id2;
static int pid_id0, pid_id1, pid_id2;
static int ddr_id1, ddr_id2;
static int pmic_reset;

static const struct of_device_id of_board_info_match[] = {
	{ .compatible = "board-info", },
	{},
};
MODULE_DEVICE_TABLE(of, of_board_info_match);

static int ver_show(struct seq_file *m, void *v)
{
	int id0, id1, id2;
	int hwid;
	char *boardver;

	id0 = gpio_get_value(hw_id0);
	id1 = gpio_get_value(hw_id1);
	id2 = gpio_get_value(hw_id2);

	hwid = (id2 << 2) + (id1 << 1) + id0;

	if (hwid == 0)
		boardver = "1.00";
	else if (hwid == 1)
		boardver = "1.01";
	else if (hwid == 2)
		boardver = "1.02";
	else
		boardver = "unknown";

	seq_printf(m, "%s\n", boardver);
	return 0;
}

static int info_show(struct seq_file *m, void *v)
{
	int id0, id1, id2;
	int pid;
	char *boardinfo;

	id0 = gpio_get_value(pid_id0);
	id1 = gpio_get_value(pid_id1);
	id2 = gpio_get_value(pid_id2);

	pid = (id2 << 2) + (id1 << 1) + id0;

	if (pid == 0)
		boardinfo = "Tinker Board 2";
	else if (pid == 1)
		boardinfo = "Tinker Board 2S - 16GB";
	else if (pid == 2)
		boardinfo = "Tinker Board 2S - 32GB";
	else
		boardinfo = "unknown";

	seq_printf(m, "%s\n", boardinfo);
	return 0;
}

static int ddr_show(struct seq_file *m, void *v)
{
	int id0, id1;
	int ddrid;
	char *ddr;

	id0 = gpio_get_value(ddr_id1);
	id1 = gpio_get_value(ddr_id2);

	ddrid = (id1 << 1) + id0;

	if (ddrid == 0)
		ddr = "2GB";
	else if (ddrid == 1)
		ddr = "4GB";
	else
		ddr = "unknown";

	seq_printf(m, "%s\n", ddr);
	return 0;
}

int pmic_restart(void)
{
	printk("pmic_reset number = %d\n", pmic_reset);
	printk("pmic_reset value1 = %d\n", gpio_get_value(pmic_reset));
	gpio_set_value(pmic_reset, 0);
	mdelay(100);
	printk("pmic_reset value2 = %d\n", gpio_get_value(pmic_reset));
	gpio_set_value(pmic_reset, 1);
	mdelay(100);
	printk("pmic_reset value3 = %d\n", gpio_get_value(pmic_reset));

	return 0;
}
EXPORT_SYMBOL(pmic_restart);

static int ver_open(struct inode *inode, struct file *file)
{
	return single_open(file, ver_show, NULL);
}

static int info_open(struct inode *inode, struct file *file)
{
	return single_open(file, info_show, NULL);
}

static int ddr_open(struct inode *inode, struct file *file)
{
	return single_open(file, ddr_show, NULL);
}

static struct file_operations boardver_ops = {
	.owner	= THIS_MODULE,
	.open	= ver_open,
	.read	= seq_read,
};

static struct file_operations boardinfo_ops = {
	.owner	= THIS_MODULE,
	.open	= info_open,
	.read	= seq_read,
};

static struct file_operations ddr_ops = {
	.owner  = THIS_MODULE,
	.open   = ddr_open,
	.read   = seq_read,
};

static int board_info_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int ret;
	struct proc_dir_entry* file;

	hw_id0 = of_get_named_gpio(dev->of_node, "hw-id0", 0);
	if (!gpio_is_valid(hw_id0)) {
		printk("No hw-id0 pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, hw_id0, GPIOF_DIR_IN, "HW_ID0");
		if (ret < 0) {
			printk("Fail to set hw-id0 pin\n");
			return ret;
		}
	}

	hw_id1 = of_get_named_gpio(dev->of_node, "hw-id1", 0);
	if (!gpio_is_valid(hw_id1)) {
		printk("No hw-id1 pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, hw_id1, GPIOF_DIR_IN, "HW_ID1");
		if (ret < 0) {
			printk("Fail to set hw-id1 pin\n");
			return ret;
		}
	}

	hw_id2 = of_get_named_gpio(dev->of_node, "hw-id2", 0);
	if (!gpio_is_valid(hw_id2)) {
		printk("No hw-id2 pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, hw_id2, GPIOF_DIR_IN, "HW_ID2");
		if (ret < 0) {
			printk("Fail to set hw-id2 pin\n");
			return ret;
		}
	}

	pid_id0 = of_get_named_gpio(dev->of_node, "pid-id0", 0);
	if (!gpio_is_valid(pid_id0)) {
		printk("No pid-id0 pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, pid_id0, GPIOF_DIR_IN, "PID_ID0");
		if (ret < 0) {
			printk("Fail to set pid-id0 pin\n");
			return ret;
		}
	}

	pid_id1 = of_get_named_gpio(dev->of_node, "pid-id1", 0);
	if (!gpio_is_valid(pid_id1)) {
		printk("No pid-id1 pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, pid_id1, GPIOF_DIR_IN, "PID_ID1");
		if (ret < 0) {
			printk("Fail to set pid-id1 pin\n");
			return ret;
		}
	}

	pid_id2 = of_get_named_gpio(dev->of_node, "pid-id2", 0);
	if (!gpio_is_valid(pid_id2)) {
		printk("No pid-id2 pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, pid_id2, GPIOF_DIR_IN, "PID_ID2");
		if (ret < 0) {
			printk("Fail to set pid-id2 pin\n");
			return ret;
		}
	}

	ddr_id1 = of_get_named_gpio(dev->of_node, "ddr-id1", 0);
	if (!gpio_is_valid(ddr_id1)) {
		printk("No ddr-id1 pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, ddr_id1, GPIOF_DIR_IN, "DDR_ID1");
		if (ret < 0) {
			printk("Fail to set ddr-id1 pin\n");
			return ret;
		}
	}

	ddr_id2 = of_get_named_gpio(dev->of_node, "ddr-id2", 0);
	if (!gpio_is_valid(ddr_id2)) {
		printk("No ddr-id2 pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, ddr_id2, GPIOF_DIR_IN, "DDR_ID2");
		if (ret < 0) {
			printk("Fail to set ddr-id2 pin\n");
			return ret;
		}
	}

	pmic_reset = of_get_named_gpio(dev->of_node, "pmic-reset", 0);
	if (!gpio_is_valid(pmic_reset)) {
		printk("No pmic_reset pin available in board-info\n");
		return -ENODEV;
	} else {
		ret = devm_gpio_request_one(dev, pmic_reset, GPIOF_OUT_INIT_LOW, "PMIC_RESET");
		if (ret < 0) {
			printk("Fail to set pmic_reset pin\n");
			return ret;
		}
	}

	file = proc_create("boardver", 0444, NULL, &boardver_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("boardinfo", 0444, NULL, &boardinfo_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("ddr", 0444, NULL, &ddr_ops);
	if (!file)
		return -ENOMEM;

	return 0;
}

static int board_info_remove(struct platform_device *pdev)
{
	gpio_free(hw_id0);
	gpio_free(hw_id1);
	gpio_free(hw_id2);

	gpio_free(pid_id0);
	gpio_free(pid_id1);
	gpio_free(pid_id2);

	gpio_free(ddr_id1);
	gpio_free(ddr_id2);

	gpio_free(pmic_reset);
	return 0;
}

static struct platform_driver boardinfo_driver = {
	.probe          = board_info_probe,
	.remove		= board_info_remove,
	.driver = {
		.name   = "board-info",
#ifdef CONFIG_OF_GPIO
		.of_match_table = of_match_ptr(of_board_info_match),
#endif
	},
};

module_platform_driver(boardinfo_driver);
