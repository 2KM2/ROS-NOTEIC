// 键盘遥控器：按键发到 /practice/cmd，rviz 节点负责执行。
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
  tio.c_lflag &= ~(ICANON | ECHO);
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 1;
  tcsetattr(STDIN_FILENO, TCSANOW, &tio);
}

void restoreMode() {
  if (g_tio_saved) tcsetattr(STDIN_FILENO, TCSANOW, &g_saved_tio);
}

void printHelp() {
  std::printf(
      "\n========= 40x40 练习包 键盘控制 =========\n"
      "  空格 / n / →   下一步\n"
      "  b / ←          上一步\n"
      "  p              播放 / 暂停\n"
      "  r              回到第 0 步\n"
      "  f              跳到最后一步\n"
      "  ] / [          加速 / 减速\n"
      "  1 / 2 / 3 / 4  BFS / DFS / Dijkstra / A*\n"
      "  v              切换 4 / 8 邻域\n"
      "  m              换一张新地图（换 seed）\n"
      "  g              用当前起终点重新跑一次\n"
      "  c              四个算法对比（没写完会显示未实现）\n"
      "  h              再打印帮助\n"
      "  q              退出\n"
      "==========================================\n");
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "practice_keyboard");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("practice/cmd", 10);

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
      if (c == 27) {
        char b1 = 0;
        char b2 = 0;
        if (read(STDIN_FILENO, &b1, 1) == 1 && b1 == '[' && read(STDIN_FILENO, &b2, 1) == 1) {
          if (b2 == 'C') send("step");
          else if (b2 == 'D') send("back");
        }
        ros::spinOnce();
        rate.sleep();
        continue;
      }
      switch (c) {
        case ' ':
        case 'n':
          send("step");
          break;
        case 'b':
          send("back");
          break;
        case 'p':
          send("play_pause");
          break;
        case 'r':
          send("reset");
          break;
        case 'f':
          send("finish");
          break;
        case ']':
          send("faster");
          break;
        case '[':
          send("slower");
          break;
        case '1':
          send("algo_bfs");
          break;
        case '2':
          send("algo_dfs");
          break;
        case '3':
          send("algo_dijkstra");
          break;
        case '4':
          send("algo_astar");
          break;
        case 'v':
          send("conn");
          break;
        case 'm':
          send("new_map");
          break;
        case 'g':
          send("replan");
          break;
        case 'c':
          send("compare");
          break;
        case 'h':
        case '?':
          printHelp();
          break;
        case 'q':
        case 3:
          restoreMode();
          std::printf("bye\n");
          return 0;
        default:
          break;
      }
    }
    ros::spinOnce();
    rate.sleep();
  }
  restoreMode();
  return 0;
}
