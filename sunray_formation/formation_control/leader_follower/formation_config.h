#include "ros_msg_utils.h"

#include <stdio.h>
#include <string.h>
#include <math.h>

#define MAX_NUM 1000  // 最大无人机数量，包括领机

// 阵型偏移配置结构体 - 存储每个从机相对于领机的偏移量
typedef struct {
    ENU_Coord offsets[MAX_NUM]; // 索引0通常为领机，偏移量为(0,0,0)
    int agent_num; // 阵型中的无人机总数
} FormationOffsets;

// 初始化一个空的阵型偏移（所有偏移量为0）
FormationOffsets createEmptyFormation(int agent_num) {
    FormationOffsets formation;
    formation.agent_num = agent_num;
    for (int i = 0; i < agent_num; i++) {
        formation.offsets[i].x = 0.0;
        formation.offsets[i].y = 0.0;
        formation.offsets[i].z = 0.0;
    }
    return formation;
}

// 创建直线队形偏移
FormationOffsets createLineFormation(int agent_num, double spacing) {
    FormationOffsets formation = createEmptyFormation(agent_num);
    for (int i = 1; i < agent_num; i++) {
        formation.offsets[i].y = -i * spacing; // 在领机后方直线排列
    }
    return formation;
}

// 创建V字形队形偏移
FormationOffsets createVFormation(int agent_num, double spacing, double angle_ratio) {
    FormationOffsets formation = createEmptyFormation(agent_num);
    for (int i = 1; i < agent_num; i++) {
        if (i % 2 == 1) {
            // 奇数编号在右侧
            formation.offsets[i].x = ((i + 1) / 2) * spacing * angle_ratio;
            formation.offsets[i].y = -((i + 1) / 2) * spacing;
        } else {
            // 偶数编号在左侧
            formation.offsets[i].x = -(i / 2) * spacing * angle_ratio;
            formation.offsets[i].y = -(i / 2) * spacing;
        }
    }
    return formation;
}

// 创建圆形队形偏移
FormationOffsets createCircleFormation(int agent_num, double radius) {
    FormationOffsets formation = createEmptyFormation(agent_num);
    for (int i = 1; i < agent_num; i++) {
        double angle = 2 * M_PI * (i - 1) / (agent_num - 1);
        formation.offsets[i].x = radius * cos(angle);
        formation.offsets[i].y = radius * sin(angle);
    }
    return formation;
}

// 创建菱形队形偏移
FormationOffsets createDiamondFormation(int agent_num, double spacing) {
    FormationOffsets formation = createEmptyFormation(agent_num);
    if (agent_num >= 5) {
        formation.offsets[1].y = spacing;  // 前方
        formation.offsets[2].x = -spacing; // 左方
        formation.offsets[3].x = spacing;  // 右方
        formation.offsets[4].y = -spacing; // 后方
        // 其他从机按直线排列
        for (int i = 5; i < agent_num; i++) {
            formation.offsets[i].y = -i * spacing;
        }
    }
    return formation;
}

// 根据偏移量计算单个从机目标位置
ENU_Coord calculateFollowerPosition(const ENU_Coord* leader_goal_point_xyz,
                                   const FormationOffsets* formation_offsets,
                                   int follower_index) {
    ENU_Coord follower_goal;
    
    if (follower_index < 0 || follower_index >= formation_offsets->agent_num) {
        // 索引无效，返回领机位置
        follower_goal = *leader_goal_point_xyz;
        printf("Wrong agent num (Total agents: %d):\n", formation_offsets->agent_num);
        return follower_goal;
    }
    
    follower_goal.x = leader_goal_point_xyz->x + formation_offsets->offsets[follower_index].x;
    follower_goal.y = leader_goal_point_xyz->y + formation_offsets->offsets[follower_index].y;
    follower_goal.z = leader_goal_point_xyz->z + formation_offsets->offsets[follower_index].z;
    
    return follower_goal;
}

// 根据偏移量计算所有从机目标位置
void calculateAllFollowerPositions(const ENU_Coord* leader_goal_point_xyz,
                                  const FormationOffsets* formation_offsets,
                                  ENU_Coord follower_goal_point_xyz[MAX_NUM]) {
    for (int i = 0; i < formation_offsets->agent_num; i++) {
        follower_goal_point_xyz[i] = calculateFollowerPosition(leader_goal_point_xyz, formation_offsets, i);
    }
}

// 设置单个无人机的偏移量
void setDroneOffset(FormationOffsets* formation, int drone_index, double x_offset, double y_offset, double z_offset) {
    if (drone_index >= 0 && drone_index < formation->agent_num) {
        formation->offsets[drone_index].x = x_offset;
        formation->offsets[drone_index].y = y_offset;
        formation->offsets[drone_index].z = z_offset;
    }
}

// 打印阵型偏移信息
void printFormationOffsets(const FormationOffsets* formation) {
    printf("Formation Offsets (Total agents: %d):\n", formation->agent_num);
    for (int i = 0; i < formation->agent_num; i++) {
        printf("  Drone %d: (%.1f, %.1f, %.1f)\n", 
               i, formation->offsets[i].x, formation->offsets[i].y, formation->offsets[i].z);
    }
}
