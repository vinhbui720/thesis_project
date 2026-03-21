#ifndef MY_ROBOT_EXAMPLE_H
#define MY_ROBOT_EXAMPLE_H

#include <memory>
#include <tesseract_examples/example.h>

namespace tesseract_environment
{
    class Environment;
}
namespace tesseract_visualization
{
    class Visualization;
}

namespace my_robot_planning
{
    class MyRobotExample : public tesseract_examples::Example
    {
    public:
        MyRobotExample(std::shared_ptr<tesseract_environment::Environment> env,
                       std::shared_ptr<tesseract_visualization::Visualization> plotter);

        bool run() override;

    private:
        bool debug_{false};
    };
}

#endif