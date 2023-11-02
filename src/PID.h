#ifndef PID_H
#define PID_H

#include <functional>

template <class T, typename U = double>
class PIDController
{
public:
  PIDController(U p, U i, U d, std::function<T()> pidSource, std::function<void(T output)> pidOutput);
  T tick();
  void setTarget(T t);
  T getTarget();
  T getOutput();
  T getFeedback();
  T getError();
  void setEnabled(bool e);
  bool isEnabled();
  T getProportionalComponent();
  T getIntegralComponent();
  T getDerivativeComponent();
  void setMaxIntegralCumulation(T max);
  T getMaxIntegralCumulation();
  T getIntegralCumulation();

  void setInputBounded(bool bounded);
  bool isInputBounded();
  void setInputBounds(T lower, T upper);
  T getInputLowerBound();
  T getInputUpperBound();
  void setOutputBounded(bool bounded);
  bool isOutputBounded();
  void setOutputBounds(T lower, T upper);
  T getOutputLowerBound();
  T getOutputUpperBound();
  void setFeedbackWrapped(bool wrap);
  bool isFeedbackWrapped();
  void setFeedbackWrapBounds(T lower, T upper);
  T getFeedbackWrapLowerBound();
  T getFeedbackWrapUpperBound();

  void setPID(U p, U i, U d);
  void setP(U p);
  void setI(U i);
  void setD(U d);
  U getP();
  U getI();
  U getD();
  void setPIDSource(T (*pidSource)());
  void setPIDOutput(void (*pidOutput)(T output));
  void registerTimeFunction(unsigned long (*getSystemTime)());

private:
  U _p;
  U _i;
  U _d;
  T target;
  T output;
  bool enabled;
  T currentFeedback;
  T lastFeedback;
  T error;
  T lastError;
  long currentTime;
  long lastTime;
  T integralCumulation;
  T maxCumulation;
  T cycleDerivative;

  bool inputBounded;
  T inputLowerBound;
  T inputUpperBound;
  bool outputBounded;
  T outputLowerBound;
  T outputUpperBound;
  bool feedbackWrapped;
  T feedbackWrapLowerBound;
  T feedbackWrapUpperBound;

  bool timeFunctionRegistered;
  std::function<T()> _pidSource;
  std::function<void(T output)> _pidOutput;
  unsigned long (*_getSystemTime)();
};

#endif