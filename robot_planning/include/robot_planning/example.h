/**
 * @file examples.h
 * @brief Examples base class
 */
#ifndef ROBOT_PLANNING_EXAMPLES_H
#define ROBOT_PLANNING_EXAMPLES_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <memory>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_environment/fwd.h>
#include <tesseract_visualization/fwd.h>
#include <tesseract_environment/environment.h>
#include <tesseract_visualization/visualization.h>

namespace Vinhtesseract_examples
{

  class Example
  {
  public:
    Example(std::shared_ptr<tesseract_environment::Environment> env,
            std::shared_ptr<tesseract_visualization::Visualization> plotter = nullptr)
        : env_(std::move(env)), plotter_(std::move(plotter))
    {
    }

    virtual ~Example() = default;
    Example(const Example &) = default;
    Example &operator=(const Example &) = default;
    Example(Example &&) = default;
    Example &operator=(Example &&) = default;

    virtual bool run() = 0;

  protected:
    std::shared_ptr<tesseract_environment::Environment> env_;
    std::shared_ptr<tesseract_visualization::Visualization> plotter_;
  };

} // namespace tesseract_examples
#endif // ROBOT_PLANNING_EXAMPLES_H