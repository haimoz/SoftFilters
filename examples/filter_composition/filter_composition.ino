#include <Arduino.h>
#include <SoftFilters.h>
#include <assert.h>

// We use a sinosoidal wave in this example.  Here are the parameters.
int delta_t = 20;
int tick = 0;  // updated with the specified delay
double amplitude = 2;  // amplitude
double omega = 0.2;  // angular velocity

double input;

// We will construct a simple filter chain that addes a timestamp,
// and then calculates the speed and acceleration.
TimestampFilter<double> tsFilter;
DifferentialFilter<double> diffFilter;
Reading<Differential<double> > diff;

FilterChain compositeFilter;

void setup()
{
  Serial.begin(38400);
  // need to pass pointers
  compositeFilter.append(&tsFilter);
  compositeFilter.append(&diffFilter);
  while (!Serial) { };
}

void loop()
{
  input = amplitude * sin(omega * tick++);
  // Note that for composite filters, the input and output need to pass in
  // their addresses.
  if (compositeFilter.push(&input, &diff)) {
    Serial.print(diff.value.position);
    Serial.print(' ');
    Serial.print(diff.value.speed * 1e6);  // 1e6 will bring the unit from per usec to per second
    Serial.print(' ');
    Serial.print(diff.value.acceleration * 1e12);  // 1e12 will bring the unit from per usec^2 to to per second^2
  }
  Serial.println();
  delay(delta_t);
}
