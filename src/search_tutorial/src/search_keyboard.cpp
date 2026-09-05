// 键盘遥控器 —— 在终端里按键控制 rviz 里的 BFS/DFS/Dijkstra 回放。
//
// 把终端设成"不回显、不等回车"（cbreak）模式，按一下键就发一条命令到 /search/cmd。
// 退出时一定要把终端设置还原，否则你的 shell 会变得没法用（打字看不见）。
//
// 比 astar_tutorial 的那个多了三类键：
//   1/2/3/a  在线切换算法（地图和起终点不变，只换 frontier 的取出规则）
//   c        打印三个算法的对比表
//   v        切换 4/8 邻域（4 邻域下 BFS 和 Dijkstra 会变成同一个东西）
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <termios.h>
#include <unistd.h>

#include <cstdio>

namespace {

termios g_saved_tio;
bool g_tio_saved = false;

void enterRawMode() {
  if (tcgetattr(STDIN_FILENO, &g_saved_tio) != 0) return;
  g_tio_saved = true;
  termios tio = g_saved_tio;
  tio.c_lflag &= ~(ICANON | ECHO);  // 关行缓冲 + 关回显
  tio.c_cc[VMIN] = 0;               // read 不阻塞等字符
  tio.c_cc[VTIME] = 1;              // 最多等 0.1s
  tcsetattr(STDIN_FILENO, TCSANOW, &tio);
}

void restoreMode() {
  if (g_tio_saved) tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_tio);
}

void printHelp() {
  std::printf(
      "\n========= BFS/DFS/Dijkstra 回放键盘控制 =========\n"
      "  空格 / n / →   下一步（终端会打印这一步的全部细节）\n"
      "  b / ←          上一步\n"
      "  p              播放 / 暂停\n"
      "  r              回到第 0 步\n"
      "  f              跳到最后一步（直接看结果）\n"
      "  ] / [          播放加速 / 减速\n"
      "  1 / 2 / 3      换算法: BFS / DFS / Dijkstra\n"
      "  a              循环切换算法\n"
      "  c              打印三个算法的对比表（同图同起终点）\n"
      "  v              切换 4 / 8 邻域（默认 4: 只上下左右，不走对角线）\n"
      "  t              切换每格上的 g / 步数 文字\n"
      "  m              换一张新地图（换 seed）\n"
      "  g              用当前起终点重新规划一次\n"
      "  h              再打印一次本帮助\n"
      "  q / Ctrl-C     退出\n"
      "-------------------------------------------------\n"
      "建议的看法: 先 1 看 BFS 一圈圈推(4 邻域下是菱形), 再 3 看 Dijkstra,\n"
      "            最后 2 看 DFS 一头扎到底; 每次都按 c 看数字。\n"
      "            4 邻域下 BFS 和 Dijkstra 的 cost 会一模一样(边代价全是 1);\n"
      "            按 v 切到 8 邻域, 再按 c, 就能看到 BFS 那行的 cost 变大。\n"
      "=================================================\n");
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "search_keyboard");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("search/cmd", 10);

  enterRawMode();
  printHelp();

  auto send = [&pub](const char* cmd) {
    std_msgs::String m;
    m.data = cmd;
    pub.publish(m);
  };

  ros::Rate rate(50);
  while (ros::ok()) {
    char c = 0;
    const ssize_t n = read(STDIN_FILENO, &c, 1);
    if (n == 1) {
      // 方向键是三字节转义序列 ESC '[' 'C'/'D'，得单独接一下
      if (c == 27) {
        char b1 = 0;
        char b2 = 0;
        if (read(STDIN_FILENO, &b1, 1) == 1 && b1 == '[' && read(STDIN_FILENO, &b2, 1) == 1) {
          if (b2 == 'C') send("step");       // →
          else if (b2 == 'D') send("back");  // ←
        }
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      switch (c) {
        case ' ': case 'n': send("step"); break;
        case 'b': send("back"); break;
        case 'p': send("play_pause"); break;
        case 'r': send("reset"); break;
        case 'f': send("finish"); break;
        case ']': send("faster"); break;
        case '[': send("slower"); break;
        case '1': send("algo_bfs"); break;
        case '2': send("algo_dfs"); break;
        case '3': send("algo_dijkstra"); break;
        case 'a': send("algo_next"); break;
        case 'c': send("compare"); break;
        case 'v': send("conn"); break;
        case 't': send("text"); break;
        case 'm': send("new_map"); break;
        case 'g': send("replan"); break;
        case 'h': case '?': printHelp(); break;
        case 'q': case 3:  // 'q' 或 Ctrl-C
          restoreMode();
          std::printf("bye\n");
          return 0;
        default: break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }

  restoreMode();
  return 0;
}
