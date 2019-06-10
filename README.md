# SoftFilters
Arduino framework and library of software data filters.
Can be used as a generic filter framework in C++.



## Quick Start

0. __Installation__

   Either install from Arduino library manager (available soon),
   or clone from Github repository to your Arduino library directory:
   ```bash
   $ cd ~/Arduino/libraries/  # enter Arduino library directory
   $ git clone https://github.com/haimoz/SoftFilters.git
   ```

1. __Include the library__

   ```C++
   #include <SoftFilters.h>
   ```

2. __Create a filter__

   This can be done in the global scope of your Arduino sketch.
   You will need to specify the data type for most filters,
   which provides more flexibility to your sketch.
   This is a C++ feature called "[templates][]"
   which is also supported in Arduino.
   ```C++
   MovingAverageFilter<double> movAvg(20);  // a 20-sample moving average filter
   ```

   [templates]: https://en.wikipedia.org/wiki/Template_(C%2B%2B)

3. __Push the data through the filter__

   Suppose your input data is in the variable `input_val`.
   And don't forget to declare an output variable to hold the output data.
   You can declare the input & output variables in the global scope
   of your Arduino sketch:
   ```C++
   double input_val, output_val;
   ```

   Call the filter's `push` method, and
   pass the input and output data by pointer.
   The `push` method will return true if there is valid output value.
   ```C++
   void loop()
   {
     ...  // suppose you have read data to the input_val variable
     if (movAvg.push(&input_val, &output_val)) {
       ...  // now you can process the output_val variable
     }
   }
   ```

4. __Speed and acceleration__

   Two types are provided for the calculation of
   speed and acceleration of your data:
   `Reading` and `Differential`.
   `Reading` is a <value, timestamp> tuple.
   `Differential` is a three-value tuple of position, speed, and acceleration.
   Using these types, you can create a `DifferentialFilter` to calculate
   the speed and acceleration.
   ```C++
   DifferentialFilter<double, usigned long> diff;

   // need to specify the value and timestamp data type
   Reading<double, unsigned long> r;

   // output of a differential filter is a `Reading` with the value type of `Differential`
   Reading<Differential<double>, unsigned long> d;

   void setup()
   {
     ...
   }
   void loop()
   {
     ...  // get the input into the reading
     if (diff.push(&r, &d)) {
       Serial.print(r.value);              Serial.print(" ");
       Serial.print(r.timestamp);          Serial.print(" ");
       Serial.print(d.value.position);     Serial.print(" ");
       Serial.print(d.value.speed);        Serial.print(" ");
       Serial.print(d.value.acceleration);
     }
     Serial.println();
     ...  // further process the output in the differential
   }
   ```

5. __Create a filter chain__

   Create a `FilterChain` to connect multiple filters.
   Then you can use it as a single filter.
   When appending the sub-filters, you need to pass by pointer.
   ```C++
   double input;
   FilterChain filters;
   MovingAverageFilter<double movAvg(20);
   DifferentialFilter<double, unsigned long> diff;
   Reading<Differential<double>, unsigned long> output;

   void setup()
   {
     ...
     filters.append(&movAvg);
     filters.append(&diff);
     ...
   }

   void loop()
   {
     ...  // get input
     if (filters.push(&input, &output)) {
       Serial.print(d.value.position);     Serial.print(" ");
       Serial.print(d.value.speed);        Serial.print(" ");
       Serial.print(d.value.acceleration); Serial.print(" ");
       Serial.print(d.timestamp);
       ...  // further process output
     }
     ...
   }
   ```



## Currently implemented filters

- `DifferentialFilter`

  Calculates the speed and acceleration.

- `TimestampFilter`

  Adds timestamp to input data, outputting the `Reading` data type.

- `MovingAverageFilter`

  Calculates the average of samples in a moving window.

- `MovingVarianceFilter`

  Calculates the variance of samples in a moving window.

- `WeightedUpdateFilter`

  Updates the output value based on a weighted average between the previous
  output and the current input.

- `AdaptiveNormalizationFilter`

  Outputs a value between 0 and 1 which is the current input normalized
  against the range of all input (including the current input).

- `OneEuroFilter`

  An adaptive low-pass filter based on the following paper:

  Géry Casiez,
  Nicolas Roussel, and
  Daniel Vogel. 2012.
  1€ Filter: A Simple Speed-based Low-pass Filter for Noisy Input
  in Interactive Systems.
  In
  _Proceedings of the SIGCHI Conference on Human Factors in Computing Systems_
  (CHI '12), 2527–2530. https://doi.org/10.1145/2207676.2208639

- `LambdaFilter`

  A filter that uses a user-supplied filter function.

- `FlowRateFilter`

  A pass-through filter for measuring the data rate.



## Documentations

For more details, see [documentations](https://haimoz.github.io/SoftFilters/) or the source code.



## Arduino examples

See examples directory.
