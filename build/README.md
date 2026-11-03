# 构建

所有命令从仓库根目录执行。源码检查只需Python 3.9+：

```bash
python -B build/check.py
```

## Vivado

使用已安装的Vivado 2026.1。`--vivado`接受完整的`vivado.bat`或`vivado`路径，未写死原作者安装目录。

```powershell
python -B build/build.py --stage project --vivado "<安装目录>/bin/vivado.bat"
python -B build/build.py --stage synth --vivado "<安装目录>/bin/vivado.bat"
python -B build/build.py --stage route --vivado "<安装目录>/bin/vivado.bat"
python -B build/build.py --stage export --vivado "<安装目录>/bin/vivado.bat"
```

`project`创建并验证Block Design；`synth`执行综合；`route`包含综合及布局布线；`export`要求已有通过检查的布线结果，检查非HDMI setup、hold和CDC后导出bit/HWH/XSA。export没有通过时不会生成可用位流。

工程输出：`src/sgm_yolo/build/lds_net_system/`；导出文件位于其`export/`。启动日志和隔离的进程缓存位于`build/runtime/`。这些路径已加入`.gitignore`。

## C++解码器

需要C++11编译器。在目标PYNQ Linux上使用设备已有g++；也可以在主机使用ARM hard-float交叉编译器：

```bash
python -B build/build.py --stage native --compiler g++ --target host
python -B build/build.py --stage native --compiler /path/to/arm-linux-gnueabihf-g++ --target armhf
```

ARM目标输出`src/sgm_yolo/dual_ov5640/liblds_net_post.so`，需与对应Python驱动一起部署。Windows主机编译的DLL只能用于本机验证。

源码发布后应在目标环境重新完成综合、时序、仿真与板端验收。软件回归通过不代表新的位流或板测已通过。
