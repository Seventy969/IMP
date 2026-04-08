#!/bin/bash
# ============================================================
# robot_undo.sh
# Removes EVERYTHING created by robot_setup_full.sh
# Safe to run — only removes robot-specific files/aliases
# ============================================================

RED='\033[0;31m'
YELLOW='\033[1;33m'
GREEN='\033[0;32m'
NC='\033[0m'

echo ""
echo "============================================================"
echo "   Lawn Mower Robot — UNDO / CLEAN-UP Script"
echo "============================================================"
echo ""
echo -e "${RED}This will remove:${NC}"
echo "  ~/robot/                          (all config, launch, scripts)"
echo "  ~/ws_lidar/src/robot_controllers/ (ROS2 package)"
echo "  ~/ws_lidar/build/robot_controllers/"
echo "  ~/ws_lidar/install/robot_controllers/"
echo "  ~/.bashrc robot aliases"
echo ""
read -p "Are you sure? (yes/no): " confirm
if [ "$confirm" != "yes" ]; then
    echo "Aborted."
    exit 0
fi

echo ""

# ── stop any running robot processes ────────────────────────
echo -e "${YELLOW}Stopping any running robot nodes...${NC}"
pkill -f motor_controller    2>/dev/null && echo "  stopped motor_controller"    || true
pkill -f cutter_controller   2>/dev/null && echo "  stopped cutter_controller"   || true
pkill -f coverage_path       2>/dev/null && echo "  stopped coverage_path_planner" || true
pkill -f sllidar_node        2>/dev/null && echo "  stopped sllidar_node"        || true
pkill -f mapping_launch      2>/dev/null && echo "  stopped mapping_launch"      || true
pkill -f navigation_launch   2>/dev/null && echo "  stopped navigation_launch"   || true
pkill -f coverage_nomap      2>/dev/null && echo "  stopped coverage_nomap"      || true
sleep 1

# ── remove ~/robot/ directory ────────────────────────────────
if [ -d ~/robot ]; then
    echo -e "${YELLOW}Removing ~/robot/ ...${NC}"
    rm -rf ~/robot
    echo -e "  ${GREEN}✓ ~/robot/ removed${NC}"
else
    echo "  ~/robot/ not found — skipping"
fi

# ── remove robot_controllers ROS2 package ───────────────────
PKG="$HOME/ws_lidar/src/robot_controllers"
if [ -d "$PKG" ]; then
    echo -e "${YELLOW}Removing robot_controllers package...${NC}"
    rm -rf "$PKG"
    rm -rf "$HOME/ws_lidar/build/robot_controllers"   2>/dev/null || true
    rm -rf "$HOME/ws_lidar/install/robot_controllers" 2>/dev/null || true
    rm -rf "$HOME/ws_lidar/log"                       2>/dev/null || true
    echo -e "  ${GREEN}✓ robot_controllers package removed${NC}"
else
    echo "  robot_controllers not found — skipping"
fi

# ── remove bashrc aliases ────────────────────────────────────
echo -e "${YELLOW}Removing bashrc aliases...${NC}"
if grep -q "# === ROBOT SHORTCUTS ===" ~/.bashrc; then
    # Delete the block between the markers (inclusive)
    sed -i '/# === ROBOT SHORTCUTS ===/,/# === END ROBOT ===/d' ~/.bashrc
    echo -e "  ${GREEN}✓ bashrc aliases removed${NC}"
else
    echo "  No robot aliases found in ~/.bashrc"
fi

# ── remove legacy files from home root ───────────────────────
echo -e "${YELLOW}Removing legacy files from home...${NC}"
for f in \
    ~/motor_controller.py \
    ~/nav2_params.yaml \
    ~/organize_robot.sh \
    ~/fix_robot_navigation.sh \
    ~/setup_robot_complete.sh; do
    [ -f "$f" ] && rm "$f" && echo "  removed $(basename $f)" || true
done

# ── optionally remove maps ────────────────────────────────────
echo ""
read -p "Remove saved maps too? (yes/no): " remove_maps
if [ "$remove_maps" = "yes" ]; then
    rm -f ~/robot/maps/*.yaml ~/robot/maps/*.pgm 2>/dev/null || true
    echo -e "  ${GREEN}✓ Maps removed${NC}"
else
    echo "  Maps preserved."
fi

echo ""
echo "============================================================"
echo -e "${GREEN}   UNDO COMPLETE.${NC}"
echo "============================================================"
echo ""
echo "Your ROS2 installation (kilted), ws_lidar workspace, and"
echo "sllidar_ros2 package are untouched."
echo ""
echo "To re-install robot files, run:"
echo "  bash robot_setup_full.sh"
echo ""
