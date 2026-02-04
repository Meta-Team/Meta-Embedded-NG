# bsp pwm

同一定时器下的PWM通道更改周期时请注意该定时器下是否有其他PWM实例，如果有请注意不要影响其他PWM通道的周期。
使用PWM DMA传输中断时注意占空比的设置，设为0将不会进入中断函数
默认tim psc已在CubeMX设置好，详情可参考CubeMX配置文件

目前只做了TIM1，2，3，12的支持，要使用其他TIM请在CubeMX中配置，同时修改PWMRegister中读取时钟频率的代码。

```
PWM_Init_Config_s pwm_config;
pwm_config.htim = &htim12;
pwm_config.channel = TIM_CHANNEL_2;
pwm_config.period = 0.001;
pwm_config.dutyratio = 0.5;
PWMInstance *pwm = PWMRegister(&pwm_config);
PWMStart(pwm);
```
