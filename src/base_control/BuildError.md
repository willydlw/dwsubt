
### Eigen/Core Build Error & Fix, Ubuntu 18.04 

The first time I included laser_geometry.h, catkin_make generated the build error shown below.

```
/opt/ros/melodic/include/laser_geometry/laser_geometry.h:47:10: fatal error: Eigen/Core: No such file or directory
 #include <Eigen/Core>
```

Internet research shows that the Ubuntu path is /usr/include/eigen3/Eigen.[1]  

Verify that is the case with your system. The image below shows the path on my system.

![Eigen/Core path](./images/eigenpath.png "Eigen/Core path")

I also verified my system had no path /usr/include/Eigen by listing everything in the usr/include directory. We will fix this issue by creating a virtual /usr/include/Eigen directory with a symbolic link.




#### The symbolic link fix

The problem is fixed by creating a symbolic link, as shown below. The symbolic link allows us to create a virtual directory or file that links to the actual physical directory or file. The general form of the command is

```
ln -s <source_file_directory> <link_file_directory>
```

In my case the physical source directory is /usr/include/eigen3/Eigen but laser_geometry.h is looking for /usr/include/Eigen. 

```
sudo ln -s /usr/include/eigen3/Eigen /usr/include/Eigen
```

The image below shows the symbolic link in the virtual directory /usr/include/Eigen.

![Eigen sym link](./images/eigenlink.png "Eigen sym link")


This fixed the catkin_make error for me.



#### References

[1] https://github.com/opencv/opencv/issues/14868 