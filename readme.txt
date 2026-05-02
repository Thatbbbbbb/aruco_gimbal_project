网络引擎文件生成

首先进入tensorrt的安装路径的bin文件夹，例
```sh
cd /home/thatbbbbbb/TensorRT-10.0.1.6/bin
```
解压文件夹中的deploy.zip将其存放到一个位置，比如主目录

然后运行下列程序，例
```sh
./trtexec --onnx=/home/thatbbbbbb/projects/aruco_gimbal_project/detect/train/weights/best.onnx --saveEngine=/home/thatbbbbbb/projects/aruco_gimbal_project/detect/train/best.engine     --staticPlugins=/home/thatbbbbbb/deploy/lib/plugin/libcustom_plugins.so     --setPluginsToSerialize=/home/thatbbbbbb/deploy/lib/plugin/libcustom_plugins.so     --fp16
```
***注意***上述的--onnx路径改成文件夹中给的onnx路径，--staticPlugins、--setPluginsToSerialize改成你放置的deploy路径，--saveEngine保存位置自行配置。
