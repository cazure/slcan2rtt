# slcan on RT-Thread

## 1、介绍

这是一个在 RT-Thread 上，用于实现slcan协议的软件包 。基于RT-Thread的device框架实现 can和serial端口的数据转换功能。

### 1.1 目录结构

| 名称 | 说明 |
| ---- | ---- |
| docs  | 文档目录 |
| examples | 例子目录，并有相应的一些说明 |
| inc  | 头文件目录 |
| src  | 源代码目录 |
|      |            |

### 1.2 许可证

slcan2rtt RT-Thread package 遵循 Apache v2.0 许可，详见 `LICENSE` 文件。

### 1.3 依赖

- RT-Thread 3.0+
- rtdevice (can、serial)

## 2、如何打开 slcan 

使用 slcan adapter package 需要在 RT-Thread 的包管理器中选择它，具体路径如下：

```
RT-Thread online packages
    miscellaneous packages --->
        [*] slcan2rtt
```

然后让 RT-Thread 的包管理器自动更新，或者使用 `pkgs --update` 命令更新包到 BSP 中。

## 3、如何使用 slcan 

在打开 slcan adapter package 后，将examples目录下测试文件复制到applications目录，修改其中的can和串口号。更多文档位于 [`/docs`](/docs) 下，使用前 **务必查看**

## 4、注意事项





## 5、联系方式 & 感谢

* 维护：chenbin182@qq.com
* 主页：https://github.com/cazure/slcan2rtt.git
