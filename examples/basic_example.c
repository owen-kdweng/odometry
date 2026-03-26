#include <stdio.h>
#include "odometry.h"

int main(void)
{
    odometry_t odom;
    double x, y, theta;
    double v, w;

    /* encoder 範圍假設是 -32768 ~ 32767 */
    if (odom_init(&odom,
                  0.05,      /* wheel radius */
                  0.30,      /* wheel base */
                  2048.0,    /* ticks per revolution */
                  -32768,    /* encoder min */
                  32767,     /* encoder max */
                  false,
                  false) != ODOM_OK)
    {
        printf("odom_init failed\n");
        return -1;
    }

    /* 
    	第一次輸入用於初始化編碼器 
		可接受非 0數值 
	*/
    odom_update(&odom, 12000, -12000, 0.01);
    
    odom_get_pose(&odom, &x, &y, &theta);
    odom_get_velocity(&odom, &v, &w);
    printf("pose: x=%.6f, y=%.6f, theta=%.6f\n", x, y, theta);
    printf("vel : v=%.6f, w=%.6f\n", v, w);
    

    /* 車體原地旋轉(順時針) */
    odom_update(&odom, 12100, -12100, 0.01);

    odom_get_pose(&odom, &x, &y, &theta);
    odom_get_velocity(&odom, &v, &w);
    printf("pose: x=%.6f, y=%.6f, theta=%.6f\n", x, y, theta);
    printf("vel : v=%.6f, w=%.6f\n", v, w);
    
    
    /* 前進 */
    odom_update(&odom, 12200, -12000, 0.01);

    odom_get_pose(&odom, &x, &y, &theta);
    odom_get_velocity(&odom, &v, &w);
    printf("pose: x=%.6f, y=%.6f, theta=%.6f\n", x, y, theta);
    printf("vel : v=%.6f, w=%.6f\n", v, w);


	/* 重置 */
    odom_reset(&odom);
    odom_get_pose(&odom, &x, &y, &theta);
    printf("after reset: x=%.6f, y=%.6f, theta=%.6f\n", x, y, theta);

    return 0;
}
