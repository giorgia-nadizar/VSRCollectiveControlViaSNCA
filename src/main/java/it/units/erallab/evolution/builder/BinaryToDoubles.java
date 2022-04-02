package it.units.erallab.evolution.builder;

import it.units.malelab.jgea.representation.sequence.bit.BitString;

import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

public class BinaryToDoubles implements PrototypedFunctionBuilder<BitString, List<Double>> {

  private final double value;

  public BinaryToDoubles(double value) {
    this.value = value;
  }

  @Override
  public Function<BitString, List<Double>> buildFor(List<Double> doubles) {
    return bitString -> {
      if (doubles.size() != bitString.size()) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of values: %d expected, %d found",
            doubles.size(),
            bitString.size()
        ));
      }
      return bitString.stream().map(b -> b ? value : -value).collect(Collectors.toList());
    };
  }

  @Override
  public BitString exampleFor(List<Double> doubles) {
    return new BitString(doubles.size());
  }
}
