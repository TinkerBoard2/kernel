#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/proc_fs.h>

static int hwid = -1, pid = -1, ddrid = -1;

static const struct of_device_id of_board_info_match[] = {
	{ .compatible = "board-info", },
	{},
};
MODULE_DEVICE_TABLE(of, of_board_info_match);

static int ver_show(struct seq_file *m, void *v)
{
	char *boardver;

	if (hwid == 0)
		boardver = "1.01";	//Due to SR using ADC, HWID0 mapping to ER.
	else if (hwid == 1)
		boardver = "1.02";	//Mapping to PR and MP.
	else if (hwid == 3)
		boardver = "1.03";
	else
		boardver = "unknown";

	seq_printf(m, "%s\n", boardver);
	return 0;
}

static int info_show(struct seq_file *m, void *v)
{
	char *boardinfo;

	if (pid == 0)
		boardinfo = "Tinker Board 2";
	else if (pid == 1)
		boardinfo = "Tinker Board 2S - 16GB";
	else if (pid == 2)
		boardinfo = "Tinker Board 2S - 32GB";
	else if (pid == 3)
		boardinfo = "Tinker Board 2 - TPS2556";
	else if (pid == 4)
		boardinfo = "Tinker Board 2S - 16GB - TPS2556";
	else
		boardinfo = "unknown";

	seq_printf(m, "%s\n", boardinfo);
	return 0;
}

static int model_show(struct seq_file *m, void *v)
{
	char *boardmodel;

	if (pid == 0 || pid == 3)
		boardmodel = "Tinker Board 2";
	else if (pid >= 1)
		boardmodel = "Tinker Board 2S";
	else
		boardmodel = "unknown";

	seq_printf(m, "%s\n", boardmodel);
	return 0;
}

static int bid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", hwid);
	return 0;
}

static int pid_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", pid);
	return 0;
}

static int ddr_show(struct seq_file *m, void *v)
{
	char *ddr;

	if (ddrid == 0)
		ddr = "2GB";
	else if (ddrid == 1)
		ddr = "4GB";
	else
		ddr = "unknown";

	seq_printf(m, "%s\n", ddr);
	return 0;
}

static int ver_open(struct inode *inode, struct file *file)
{
	return single_open(file, ver_show, NULL);
}

static int info_open(struct inode *inode, struct file *file)
{
	return single_open(file, info_show, NULL);
}

static int model_open(struct inode *inode, struct file *file)
{
	return single_open(file, model_show, NULL);
}

static int bid_open(struct inode *inode, struct file *file)
{
	return single_open(file, bid_show, NULL);
}

static int pid_open(struct inode *inode, struct file *file)
{
	return single_open(file, pid_show, NULL);
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

static struct file_operations boardmodel_ops = {
	.owner	= THIS_MODULE,
	.open	= model_open,
	.read	= seq_read,
};

static struct file_operations boardid_ops = {
	.owner	= THIS_MODULE,
	.open	= bid_open,
	.read	= seq_read,
};

static struct file_operations projectid_ops = {
	.owner	= THIS_MODULE,
	.open	= pid_open,
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

	int hw_id0, hw_id1, hw_id2;
	int pid_id0, pid_id1, pid_id2;
	int ddr_id1, ddr_id2;

	int id0, id1, id2;

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
	id0 = gpio_get_value(hw_id0);

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
	id1 = gpio_get_value(hw_id1);

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
	id2 = gpio_get_value(hw_id2);

	hwid = (id2 << 2) + (id1 << 1) + id0;

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
	id0 = gpio_get_value(pid_id0);

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
	id1 = gpio_get_value(pid_id1);

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
	id2 = gpio_get_value(pid_id2);

	pid = (id2 << 2) + (id1 << 1) + id0;

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
	id1 = gpio_get_value(ddr_id1);

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
	id2 = gpio_get_value(ddr_id2);

	ddrid = (id2 << 1) + id1;

	gpio_free(hw_id0);
	gpio_free(hw_id1);
	gpio_free(hw_id2);

	gpio_free(pid_id0);
	gpio_free(pid_id1);
	gpio_free(pid_id2);

	gpio_free(ddr_id1);
	gpio_free(ddr_id2);

	file = proc_create("boardver", 0444, NULL, &boardver_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("boardinfo", 0444, NULL, &boardinfo_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("boardmodel", 0444, NULL, &boardmodel_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("boardid", 0444, NULL, &boardid_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("projectid", 0444, NULL, &projectid_ops);
	if (!file)
		return -ENOMEM;

	file = proc_create("ddr", 0444, NULL, &ddr_ops);
	if (!file)
		return -ENOMEM;

	return 0;
}

int get_board_id(void)
{
	return hwid;
}
EXPORT_SYMBOL_GPL(get_board_id);

int get_project_id(void)
{
	return pid;
}
EXPORT_SYMBOL_GPL(get_project_id);

static int board_info_remove(struct platform_device *pdev)
{
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
