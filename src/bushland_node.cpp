#include "hestia/bushland.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "bushland");
    Bushland bushland;

    // 添加Bush对象到Map
    bushland.addBush(Bush(0, {1.0, 2.0}, false, 0));
    bushland.addBush(Bush(1, {2.0, 3.0}, false, 0));
    bushland.addBush(Bush(2, {3.0, 4.0}, false, 0));
    bushland.addBush(Bush(3, {3.0, 3.0}, false, 0));

    // 添加Reservoir对象到Map
    bushland.addReservoir(Reservoir(1, {4.0, 5.0}));

    // // 调用saveAndUpdate函数
    // bushland.saveAndUpdate();

    ros::spin();

    return 0;
}