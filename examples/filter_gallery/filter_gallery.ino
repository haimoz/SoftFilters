#include <Arduino.h>
#include <SoftFilters.h>

// We use a sinosoidal wave in this example.  Here are the parameters.
int delta_t = 20;
int tick = 0;  // updated with the specified delay
double amplitude = 2;  // amplitude
double omega = 0.2;  // angular velocity

double input;

MovingAverageFilter<double, double> movAvg(20);
double avg;

MovingVarianceFilter<double, double> movVar(20);
double var;

TimestampFilter<double> tsFilter;
Reading<double> reading;

DifferentialFilter<double> diffFilter;
Reading<Differential<double> > diff;

WeightedUpdateFilter<double, double> wtUpd(0.5);
double weighted_val;

AdaptiveNormalizationFilter<double> adpNorm;
double normalized_val;

OneEuroFilter<double, unsigned long> oneEuro(60, 10, 0.001, 40);
Reading<double> one_euro_reading;

bool add1(double const &in, double &out)
{
  out = in + 1;
  return true;
}
LambdaFilter<double, double> lambda(add1);
double lambda_val;

FlowRateFilter<double> flow;

void setup()
{
  Serial.begin(38400);
  while (!Serial) { };
}

void loop()
{
  input = amplitude * sin(omega * tick++);
  Serial.print(input, 3);
  Serial.print(' ');
  if (movAvg.push(&input, &avg)) {
    Serial.print(avg, 3);
    Serial.print(' ');
  }
  if (movVar.push(&input, &var)) {
    Serial.print(var, 3);
    Serial.print(' ');
  }
  tsFilter.push(&input, &reading);
  if (diffFilter.push(&reading, &diff)) {
    Serial.print(diff.value.speed * 1e6, 3);  // 1e6 will bring the unit from per usec to per second
    Serial.print(' ');
    Serial.print(diff.value.acceleration * 1e12, 3);  // 1e12 will bring the unit from per usec^2 to to per second^2
    Serial.print(' ');
  }
  if (wtUpd.push(&input, &weighted_val)) {
    Serial.print(weighted_val, 3);
    Serial.print(' ');
  }
  if (adpNorm.push(&input, &normalized_val)) {
    Serial.print(normalized_val, 3);
    Serial.print(' ');
  }
  if (oneEuro.push(&reading, &one_euro_reading)) {
    Serial.print(one_euro_reading.value, 3);
    Serial.print(' ');
  }
  if (lambda.push(&input, &lambda_val)) {
    Serial.print(lambda_val, 3);
    Serial.print(' ');
  }
  if (flow.push(&input, NULL)) {
    Serial.print(flow.get_flow_rate(), 3);
  }
  Serial.println();
  delay(delta_t);
}
