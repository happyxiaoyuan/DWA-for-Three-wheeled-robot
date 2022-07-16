# DWA-for-Three-wheeled-robot
create a Three-wheeled robot in webots and use DWA for obstacle avoidance 
在webpts中创建一个世界和三轮机器人（为了简化过程，使用supervisor中的方法或取机器人的位置信息和角度信息），在使用DWA算法进行避障之前需要做一些准备工作，那就是获取地图上的障碍物位置信息。你需要通过controller中的getmap.py让机器人进行栅格地图绘制，再通过get_object.py提取地图中的障碍物信息，注意getmap.py，get_object.py还有ab_avoid.py中使用的文件路径要和实际具体路径相匹配。完成准备工作后就可以在controller中选择ab_avoid.py完成避障规划任务。
如果自己搭建了世界和机器人，可能存在传感器名字的不同。如果更改了障碍物，那么可能需要调整DWA算法的三个增益，这将是一个试凑的过程。
