package it.units.erallab.evolution.builder;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

public class TernaryToDoubles implements PrototypedFunctionBuilder<List<Integer>, List<Double>> {

  private final double value;

  public TernaryToDoubles(double value) {
    this.value = value;
  }

  @Override
  public Function<List<Integer>, List<Double>> buildFor(List<Double> doubles) {
    return integers -> {
      if (doubles.size() != integers.size()) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of values: %d expected, %d found",
            doubles.size(),
            integers.size()
        ));
      }
      return integers.stream().map(i -> (i - 1) * value).collect(Collectors.toList());
    };
  }

  @Override
  public List<Integer> exampleFor(List<Double> doubles) {
    return doubles.stream().map(d -> 3).collect(Collectors.toList());
  }

}
