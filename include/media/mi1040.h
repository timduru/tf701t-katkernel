#ifndef __MI1040_H__
#define __MI1040_H__

struct mi1040_power_rail {
	struct regulator *vddio_1v8;
	struct regulator *avdd_2v8;
};

#ifdef __KERNEL__
struct mi1040_platform_data {
	int (*power_on)(struct mi1040_power_rail *);
	int (*power_off)(struct mi1040_power_rail *);
};
#endif /* __KERNEL__ */

#endif  /* __MI1040_H__ */
