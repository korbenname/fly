#!/usr/bin/env python3
"""第5步真实ROS故障矩阵入口：复用既有驱动，启用完整故障下降。

不生成、不修改或exec驱动源码。控制算法、节点ELF和所有运行参数仍由
ros_stage3_lifecycle校验；本入口仅为11项矩阵选择水平候选和完整下降选项。
"""
import sys

from ros_stage3_lifecycle import main as lifecycle_main

SCENARIOS = ('track_lost', 'track_timeout', 'track_source', 'track_reset', 'track_origin',
             'horizontal_feedback_invalid', 'horizontal_reset', 'track_reordered',
             'track_duplicate', 'track_multisource', 'preheight_flood')


def main(argv=None):
    arguments = list(sys.argv[1:] if argv is None else argv)
    return lifecycle_main(['--horizontal', '--complete-fault-descent', *arguments])


if __name__ == '__main__':
    raise SystemExit(main())
