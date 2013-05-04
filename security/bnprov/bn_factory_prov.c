/*
 * Copyright (c) 2010 Barnes & Noble.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 */
#include <linux/kobject.h>
#include <linux/init.h>

#define HDCP_MASK ( 1 << 1 )
#define WV_MASK	  ( 1 << 0 )

static char prov_data = 0;


ssize_t bnprov_wv_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", ( prov_data & WV_MASK ) );
}
ssize_t bnprov_hdcp_show(struct kobject * kobj, struct kobj_attribute * attr, char * buf)
{
	return sprintf(buf, "%d\n", ( ( prov_data & HDCP_MASK ) >> 1 ));
}

ssize_t bnprov_data_store(struct kobject * kobj, struct kobj_attribute * attr, const char * buf, size_t size)
{
	prov_data |= *buf;
	return size;
}

static struct kobj_attribute module_store_attribute =
	__ATTR(data, 0664, NULL, bnprov_data_store);

static struct kobj_attribute module_wv_show_attribute =
	__ATTR(wv, 0664, bnprov_wv_show, NULL);

static struct kobj_attribute module_hdcp_show_attribute =
	__ATTR(hdcp, 0664, bnprov_hdcp_show, NULL);


static struct attribute * bnprov_attributes[] = {
	&module_store_attribute.attr,
	&module_wv_show_attribute.attr,
	&module_hdcp_show_attribute.attr,
	NULL,
};

static struct attribute_group bnprov_attr_group = {
	.attrs = bnprov_attributes,
};

static struct kobject *bnprov_kobj;

static int bnprov_init(void)
{
	int ret = -EPERM;


	bnprov_kobj = kobject_create_and_add("bnprov", NULL);
	if (bnprov_kobj) {
		ret = sysfs_create_group(bnprov_kobj, &bnprov_attr_group);
	}

	return ret;
}

static void bnprov_exit(void)
{
	sysfs_remove_group(bnprov_kobj, &bnprov_attr_group);
}

module_init(bnprov_init);
module_exit(bnprov_exit);
