# C++ 编译运行方式

如果安装了 `g++`，可以这样编译：

```bash
g++ bfs.cpp -std=c++17 -O2 -o bfs
./bfs
```

Windows 下：

```bash
g++ bfs.cpp -std=c++17 -O2 -o bfs.exe
bfs.exe
```

其他文件同理：

```bash
g++ dfs.cpp -std=c++17 -O2 -o dfs
g++ ucs.cpp -std=c++17 -O2 -o ucs
g++ greedy.cpp -std=c++17 -O2 -o greedy
g++ astar.cpp -std=c++17 -O2 -o astar
```
