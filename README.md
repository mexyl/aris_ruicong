# Aris
Aris为潘阳的个人项目，主要由潘阳独立开发（除了tinyxml2的部分，该部分由Lee Thomason等人开发，参考https://github.com/leethomason/tinyxml2 在此向原作者表示感谢），同时参考和吸收了上海交通大学徐奕林、柴勋和田园的一些经验。
本项目被用于驱动上海交通大学重大装备研究所的六足机器人与三支链机械臂。

项目基于Xenomai-Linux实时系统，需要依赖Etherlab库，除此之外再无依赖。
项目使用c++语言，采用了大量的c++11的特性，需要基于gcc4.8以后的编译器。windows版需要vs2015，在windows下无法使用Ethercat实时控制功能。

项目主要功能包含五部分：
1、aris::core部分，这部分主要包含主体框架，消息循环、管道通讯、实时与非实时之间的通讯、socket通讯、可以识别矩阵的表达式计算器、xml解析等功能。
2、aris::dynamic部分，这部分主要包含动力学框架，支持使用xml文件编写动力学模型、自动生成adams仿真模型，高速仿真模型（用于实时系统），螺旋理论，规划函数等功能。
3、aris::control部分，这部分只可在linux实时系统下运行，可以自动解析xml文件，生成Ethercat网络拓扑、并可以控制Elmo驱动器以及ati传感器。
4、aris::sensor部分，这部分为传感器框架，已经集成好xsens陀螺仪，正在集成kinect，这个框架主要用来高速传输传感器数据到实时循环内。
5、aris::server部分，这部分自动生成一个server，运行在机器人控制器中，可以让用户自定义指令及对应的步态等。
# aris_ruicong
