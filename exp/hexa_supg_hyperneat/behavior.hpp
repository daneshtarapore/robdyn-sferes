#ifndef BEHAVIOR_HPP
#define BEHAVIOR_HPP


struct Behavior
{
    std::vector<float> position;
    float drift;
    float transferability;
    float performance;
    float arrivalangle;
    float direction;

    std::vector<float>  features;

    std::vector<float>  features_simu1, features_simu2, features_simu3;

};
#endif
