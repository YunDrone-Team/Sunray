
// 示例使用函数
int main() {
    ENU_Coord leader_goal = {100.0, 200.0, 50.0};
    ENU_Coord follower_positions[MAX_NUM];
    
    // 示例1：创建5架无人机的V字形队形
    printf("=== Example 1: 5-Agent V Formation ===\n");
    int agent_num = 5;
    FormationOffsets v_formation = createVFormation(agent_num, 10.0, 1.0);
    printFormationOffsets(&v_formation);
    
    // 分别计算每个从机的位置
    for (int i = 1; i < agent_num; i++) {
        ENU_Coord pos = calculateFollowerPosition(&leader_goal, &v_formation, i);
        printf("Follower %d goal: (%.1f, %.1f, %.1f)\n", i, pos.x, pos.y, pos.z);
    }
    
    // 示例2：创建3架无人机的直线队形
    printf("\n=== Example 2: 3-Agent Line Formation ===\n");
    agent_num = 3;
    FormationOffsets line_formation = createLineFormation(agent_num, 8.0);
    printFormationOffsets(&line_formation);
    
    // 计算所有从机位置
    calculateAllFollowerPositions(&leader_goal, &line_formation, follower_positions);
    for (int i = 0; i < agent_num; i++) {
        printf("Drone %d goal: (%.1f, %.1f, %.1f)\n", 
               i, follower_positions[i].x, follower_positions[i].y, follower_positions[i].z);
    }
    
    // 示例3：动态修改队形
    printf("\n=== Example 3: Dynamic Formation Change ===\n");
    agent_num = 4;
    FormationOffsets dynamic_formation = createCircleFormation(agent_num, 15.0);
    
    printf("Initial circle formation:\n");
    for (int i = 1; i < agent_num; i++) {
        ENU_Coord pos = calculateFollowerPosition(&leader_goal, &dynamic_formation, i);
        printf("Follower %d: (%.1f, %.1f, %.1f)\n", i, pos.x, pos.y, pos.z);
    }
    
    // 动态修改为直线队形
    printf("\nChanged to line formation:\n");
    dynamic_formation = createLineFormation(agent_num, 8.0);
    for (int i = 1; i < agent_num; i++) {
        ENU_Coord pos = calculateFollowerPosition(&leader_goal, &dynamic_formation, i);
        printf("Follower %d: (%.1f, %.1f, %.1f)\n", i, pos.x, pos.y, pos.z);
    }
    
    return 0;
}