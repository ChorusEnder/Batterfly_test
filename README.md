# 基于stm32开发的仿生蝴蝶项目

电机版系统框图
```mermaid
graph

A(stm32f103)
B[Tmag5273]
C[Drv8837]
D[富斯A8s接收机]
E[电机]

B --> |iic| A
D --> |UART| A
A --> |PWM| C
C --> |驱动| E
E --> |角度测量| B

```
