#!/usr/bin/env bash
# 准备容器可用的 X11 授权文件 /tmp/.docker.xauth
#
# 兼容三种宿主机情况:
#   1. Xorg 本地会话   -> cookie 在 $XAUTHORITY 或 ~/.Xauthority
#   2. GNOME/Wayland   -> :0 其实是 Xwayland,cookie 在 /run/user/$UID/.mutter-Xwaylandauth.XXXXXX
#                         (这个文件名每次开机都会变,且旧文件会残留,所以必须逐个验证)
#   3. SSH X11 转发    -> DISPLAY=localhost:10.0,cookie 在 ~/.Xauthority(容器需 --network host)
#
# 可以单独执行:  ./x11_setup.sh
# 也被 run.sh 自动 source。

XAUTH_OUT=/tmp/.docker.xauth

# ---------- 1. 确定 DISPLAY ----------
if [ -z "${DISPLAY:-}" ]; then
  # 优先从正在运行的 Xwayland / Xorg 进程里读它服务的 display 号
  DETECTED=$(ps -u "$(id -u)" -o args= 2>/dev/null \
             | grep -oE '(Xwayland|Xorg) +:[0-9]+' \
             | grep -oE ':[0-9]+' | head -1)
  # 退而求其次:扫描属于当前用户的 X socket(排除 gdm 登录屏的 :1024+)
  if [ -z "$DETECTED" ]; then
    for s in /tmp/.X11-unix/X*; do
      [ -S "$s" ] || continue
      n=${s##*/X}
      [ "$n" -ge 1024 ] 2>/dev/null && continue
      [ -O "$s" ] || continue
      DETECTED=":$n"
      break
    done
  fi
  if [ -n "$DETECTED" ]; then
    export DISPLAY="$DETECTED"
    echo "[x11] DISPLAY 未设置,自动检测为 $DISPLAY"
  else
    echo "[x11] 警告: 找不到可用的 X display,GUI(rviz)将无法启动" >&2
    export DISPLAY=":0"
  fi
fi

# ---------- 2. 收集候选 xauth 文件(按优先级) ----------
CANDIDATES=()
[ -n "${XAUTHORITY:-}" ] && [ -f "${XAUTHORITY}" ] && CANDIDATES+=("$XAUTHORITY")

# GNOME/Wayland: 从 Xwayland 进程的 -auth 参数拿到当前会话真正在用的那个文件
MUTTER_AUTH=$(ps -u "$(id -u)" -o args= 2>/dev/null \
              | grep -oE '\-auth +[^ ]*mutter-Xwaylandauth[^ ]*' \
              | awk '{print $2}' | head -1)
[ -n "$MUTTER_AUTH" ] && [ -f "$MUTTER_AUTH" ] && CANDIDATES+=("$MUTTER_AUTH")

[ -f "$HOME/.Xauthority" ] && CANDIDATES+=("$HOME/.Xauthority")

# 兜底:残留的 mutter auth 文件(新的在前)。上面几个都没命中时才靠它们碰运气,
# 所以下面会用 xset 逐个验证,过期的直接丢掉。
while IFS= read -r f; do
  [ -n "$f" ] && CANDIDATES+=("$f")
done < <(ls -t /run/user/"$(id -u)"/.mutter-Xwaylandauth.* 2>/dev/null)

# ---------- 3. 挑出真正能连上 $DISPLAY 的那个,写成 FamilyWild(ffff) ----------
# 必须只写有效 cookie:同一个 display 塞多条不同 cookie 会让 Xlib 用错那条。
rm -f "$XAUTH_OUT"
touch "$XAUTH_OUT"
chmod 644 "$XAUTH_OUT"

GOOD_FILE=""
CAN_VALIDATE=0
command -v xset >/dev/null 2>&1 && CAN_VALIDATE=1

for f in "${CANDIDATES[@]}"; do
  [ -s "$f" ] || continue
  [ -z "$(XAUTHORITY="$f" xauth nlist 2>/dev/null)" ] && continue
  if [ "$CAN_VALIDATE" -eq 1 ]; then
    if XAUTHORITY="$f" DISPLAY="$DISPLAY" xset q >/dev/null 2>&1; then
      GOOD_FILE="$f"
      break
    fi
  else
    # 没有 xset 可用,只能相信优先级最高的那个
    GOOD_FILE="$f"
    break
  fi
done

if [ -n "$GOOD_FILE" ]; then
  XAUTHORITY="$GOOD_FILE" xauth nlist 2>/dev/null \
    | sed -e 's/^..../ffff/' \
    | xauth -f "$XAUTH_OUT" nmerge - 2>/dev/null || true
fi

chmod 644 "$XAUTH_OUT"

if [ ! -s "$XAUTH_OUT" ]; then
  echo "[x11] 警告: 没能提取到可用的 X11 cookie,$XAUTH_OUT 为空。" >&2
  echo "[x11]        rviz 会报 'Authorization required'。请在宿主机图形会话的终端里执行:" >&2
  echo "[x11]            xhost +local:" >&2
else
  echo "[x11] DISPLAY=$DISPLAY  cookie 来源: $GOOD_FILE"
  echo "[x11] 已写入 $(xauth -f "$XAUTH_OUT" nlist 2>/dev/null | wc -l) 条 -> $XAUTH_OUT"
fi

export XAUTH_FILE="$XAUTH_OUT"
