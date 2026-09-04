// 键盘遥控器 —— 在终端里按键控制 rviz 里的 A* 回放。
//
// 把终端设成"不回显、不等回车"（cbreak）模式，按一下键就发一条命令到 /astar/cmd。
// 退出时一定要把终端设置还原，否则你的 shell 会变得没法用（打字看不见）。
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
      "\n============== A* 回放键盘控制 ==============\n"
      "  空格 / n   下一步（单步，终端会打印这一步的全部细节）\n"
      "  b          上一步\n"
      "  p          播放 / 暂停\n"
      "  r          回到第 0 步\n"
      "  f          跳到最后一步（直接看结果）\n"
      "  ] / [      播放加速 / 减速\n"
      "  t          切换 g/h/f 文字标签\n"
      "  m          换一张新地图（换 seed）\n"
      "  c          用当前起终点重新规划一次\n"
      "  h          再打印一次本帮助\n"
      "  q / Ctrl-C 退出\n"
      "============================================\n");
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "astar_keyboard");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::String>("astar/cmd", 10);

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
      switch (c) {
        case ' ': case 'n': send("step"); break;
        case 'b': send("back"); break;
        case 'p': send("play_pause"); break;
        case 'r': send("reset"); break;
        case 'f': send("finish"); break;
        case ']': send("faster"); break;
        case '[': send("slower"); break;
        case 't': send("text"); break;
        case 'm': send("new_map"); break;
        case 'c': send("replan"); break;
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
