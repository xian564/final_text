# 01 · 编译错误：`endl` 未声明

- **版本**：用户替换后的 `src/detection_node.cpp`（365 行，未提交）
- **命令**：
  ```bash
  source /opt/ros/noetic/setup.bash && cd /root/catkin_ws && catkin_make
  ```

## 报错原文

```
/root/catkin_ws/src/final_text/src/detection_node.cpp:125:34: error: 'endl' was not declared in this scope; did you mean 'std::endl'?
  125 |             std::cout << "ok" << endl;
      |                                  ^~~~
/usr/include/c++/9/ostream:599:5: note: 'std::endl' declared here
make[2]: *** [final_text/CMakeFiles/detect_node.dir/build.make:63: ...detection_node.cpp.o] Error 1
make[1]: *** [CMakeFiles/Makefile2:4392: final_text/CMakeFiles/detect_node.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
Invoking "make -j16 -l16" failed
```

## 原因

文件只 `#include <iostream>`，没有 `using namespace std;`；`std::cout` 写了全名，但 `endl` 没写。

## 处理

用户手动把 `endl` 改为 `std::endl`（源码 mtime 12:57:32），未改动其他逻辑。

## 结果

重新编译通过：
```
[100%] Linking CXX executable /root/catkin_ws/devel/lib/final_text/detect_node
[100%] Built target detect_node
```

## 遗留

无。该错误只影响编译，不影响运行时逻辑。
