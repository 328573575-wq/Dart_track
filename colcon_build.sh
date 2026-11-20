#!/usr/bin/env bash
# 以 --symlink-install 模式构建整个工作区, 构建完可选 source。
#
# 用法:
#   source colcon_build.sh    # 推荐: 构建 + 把环境带进当前终端
#   bash colcon_build.sh      # 只构建; 完成后需手动 source install/setup.bash
#
# 特性:
#   - 只有当前构建缓存不是 symlink 模式时才全量清理重建;
#     已经是 symlink 模式则走增量构建, 不会每次 rm -rf build install。
#   - symlink 模式下 install/config.yaml 是指向 src 的软链接,
#     改 src/params_control/config.yaml 后重启节点即生效, 无需重新构建。

set -e

# 无论从哪里调用, 都先切到脚本所在目录（工作区根目录）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 清掉当前 shell 里残留的工作区路径（比如之前 source install/setup.bash 留下的 install/*）。
# 否则 colcon 会报一堆 "path doesn't exist" 警告（无害但很吵）。
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH
source /opt/ros/humble/setup.bash

# PATH 里 nvm 的 node 目录带有 npm 包（如 @wcldyx/claude-code-switcher）提供的同名 cc，
# CMake 编译器检测按 PATH 找到它会报 "error: unknown option '-o'"，
# 显式指定系统编译器绕开；尊重外部已设置的值。
export CC="${CC:-/usr/bin/gcc}"
export CXX="${CXX:-/usr/bin/g++}"

# 判断是否已经是 symlink 模式:
#   全部已构建包的 CMakeCache 都带 AMENT_CMAKE_SYMLINK_INSTALL:BOOL=ON 才算;
#   否则（混用 / 没有缓存 / 部分非 symlink）就要全量清理重建,
#   避免 symlink 安装覆盖 install 里已有真实目录时报 "Is a directory"。
need_clean=0
caches=(build/*/CMakeCache.txt)
if [ -d build ] && [ -e "${caches[0]}" ]; then
  for cache in "${caches[@]}"; do
    # 缓存里布尔值可能是 ON 或 1, 两种都认
    if ! grep -Eq '^AMENT_CMAKE_SYMLINK_INSTALL:BOOL=(ON|1)' "$cache"; then
      need_clean=1
      break
    fi
  done
else
  need_clean=1
fi

if [ "$need_clean" = "1" ]; then
  echo ">>> 构建缓存非 symlink 模式, 全量清理重建"
  rm -rf build install log
else
  echo ">>> 已是 symlink 模式, 增量构建"
fi

if ! colcon build --symlink-install; then
  echo "构建失败, 请查看上方错误日志。" >&2
  # 被 source 时优雅返回, 不要把用户终端顶掉
  return 1 2>/dev/null || exit 1
fi

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # 被直接运行: 子 shell 的 source 带不进当前终端
  echo ">>> 构建完成。请在当前终端执行: source install/setup.bash"
else
  source install/setup.bash
  echo ">>> 构建完成, 工作区环境已 source。"
  echo ">>> 修改 src/params_control/config.yaml 后重启节点即可生效。"
fi
