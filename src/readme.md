# 项目3 轨迹规划 代码说明

这个readme是markdown格式的，网上找一个markdown在线编辑器，把文本粘贴进去就能看到格式了。

## 总览

根src目录下有三个ros包。
一、*grid_path_searcher*文件夹下的是你们主要要看的部分，其中*random_complex_generator.cpp*是生成随机地图的，你们可以不管它。*demo_node.cpp*是整个程序的入口文件，负责调用你们写的A\*算法，*Astar_searcher.cpp*是主要写你们代码的地方，这里面已经有一点函数了，是处理障碍物信息的，障碍物信息存储在*data*变量中。这个包里的几个文件的代码建议提前读一读，就几百行。
二、*rviz_plugins*文件夹下的文件，完全不用管它，他就是一个小工具。其用处是可以让你在rviz中用鼠标选取一个点，这个点会通过`/goal`消息发出来，可以使你任意选取轨迹规划的末端点。
三、*waypoint_generator*文件夹。这里可以生成八字、圆形、手动给点等一系列空间点，也负责将`/goal`命令转发到*demo_node*节点。你们后面生成多段minimum snap轨迹的时候会用到。具体用的时候读一遍代码吧，不到三百行。

## 运行方式

在工作空间目录下输入`source devel/setup.bash`执行环境配置文件，随后执行 `roslaunch grid_path_searcher demo.launch`即可。

## 一些提醒
启动程序之后建议先用`rqt_graph`看看节点与节点之间是哪些topic在通信，再用`rostopic list`看看都有哪些topic，用`rostopic echo <topic name>`看看topic都在往外发啥。
记得每个窗口都要`source devel/setup.bash`。
