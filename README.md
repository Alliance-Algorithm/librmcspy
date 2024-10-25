# LibRMCS

librmcs 是 [无下位机控制系统 RMCS（RoboMaster Control System）](https://github.com/Alliance-Algorithm/RMCS) 的一个高度精简版本。
它移除了所有可移除的依赖，只保留核心通讯部分。

## 为什么选择 LibRMCS?

相对传统下位机控制：

- 更低难度。依托良好的封装特性，librmcs 大幅降低了使用者的开发难度。开发者不需要考虑中断、寄存器、时钟树等嵌入式相关知识，不需要学习 HAL 和 STM32CubeMX。只需要拥有基础的语言知识，即可快速开发各类控制程序。

  这也为非电控组成员（如机械/视觉）编写简单的测试用控制代码提供了可能。使用者可以在任意非实时操作系统（Windows/Linux）上，使用任意语言（C++/Python）编写和调试控制代码。
  甚至，在现代 AI 工具的加持下，即使没有任何编程语言基础，也可以快速地编写出可用的控制代码。

- 更多可用资源。由于代码在桌面计算机运行，librmcs 可以借助桌面计算机的算力和储存能力。
  例如，在需要对机构（如云台）进行参数整定时，可借助 librmcs 编写测试用控制代码，将收集的各项信息直接放入硬盘中的 csv 文件中，方便导入 Matlab 等软件进行进一步分析。

- 更良好的语言生态。作为一个轻量级无下位机控制库，librmcs 也可以与各编程语言的生态完美结合。
  例如，对机械新造出的待测试机构，可借助 librmcs 编写测试用控制代码，并将测试结果通过 python 的 requests 库通过网络实时上传至飞书等协作软件进行信息收集和共享。

相对无下位机控制系统 RMCS：

- 易于学习。
  RMCS 为了实现同一套代码对全兵种的完整控制，代码量较大，增加了一些对初学者较为复杂的概念（组件、子组件、插件、依赖分析、更新顺序等），这保证了代码在架构上统一和整洁，但为框架的初学者带来了较大的思维负担。
  librmcs 正为了简化使用者学习而生。它移除了复杂的概念，将简洁的核心接口直接暴露给使用者，给予了更大的自由度的同时，大幅平滑了学习曲线。

- 极其轻量和跨平台。为了开发的便利性，RMCS 对 ROS2、OpenCV、Eigen 等工具有强依赖，而为了方便部署，RMCS 在开发环境和部署环境均采用了 docker 容器。这限制了 RMCS 能使用的操作系统和语言，使用者必须在 linux 系统上使用 C++ 语言进行开发。
  librmcs 通过移除所有可移除的依赖解决了这个问题，并使得它比 RMCS 更适合于小规模的快速开发（如测试项目、数据收集等）。