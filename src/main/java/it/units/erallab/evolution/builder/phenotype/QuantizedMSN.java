package it.units.erallab.evolution.builder.phenotype;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedMultilayerSpikingNetwork;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedMultivariateSpikingFunction;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedSpikingFunction;

import java.util.Collections;
import java.util.List;
import java.util.function.BiFunction;
import java.util.function.Function;

/**
 * @author eric
 */
public class QuantizedMSN implements PrototypedFunctionBuilder<List<Double>, QuantizedMultivariateSpikingFunction> {

  private final double innerLayerRatio;
  private final int nOfInnerLayers;
  private final BiFunction<Integer, Integer, QuantizedSpikingFunction> neuronBuilder;


  public QuantizedMSN(double innerLayerRatio, int nOfInnerLayers, BiFunction<Integer, Integer, QuantizedSpikingFunction> neuronBuilder) {
    this.innerLayerRatio = innerLayerRatio;
    this.nOfInnerLayers = nOfInnerLayers;
    this.neuronBuilder = neuronBuilder;
  }

  private int[] innerNeurons(int nOfInputs, int nOfOutputs) {
    int[] innerNeurons = new int[nOfInnerLayers];
    int centerSize = (int) Math.max(2, Math.round(nOfInputs * innerLayerRatio));
    if (nOfInnerLayers > 1) {
      for (int i = 0; i < nOfInnerLayers / 2; i++) {
        innerNeurons[i] = nOfInputs + (centerSize - nOfInputs) / (nOfInnerLayers / 2 + 1) * (i + 1);
      }
      for (int i = nOfInnerLayers / 2; i < nOfInnerLayers; i++) {
        innerNeurons[i] = centerSize + (nOfOutputs - centerSize) / (nOfInnerLayers / 2 + 1) * (i - nOfInnerLayers / 2);
      }
    } else if (nOfInnerLayers > 0) {
      innerNeurons[0] = centerSize;
    }
    return innerNeurons;
  }

  @Override
  public Function<List<Double>, QuantizedMultivariateSpikingFunction> buildFor(QuantizedMultivariateSpikingFunction function) {
    return values -> {
      int nOfInputs = function.getInputDimension();
      int nOfOutputs = function.getOutputDimension();
      int[] innerNeurons = innerNeurons(nOfInputs, nOfOutputs);
      int nOfWeights = QuantizedMultilayerSpikingNetwork.countWeights(nOfInputs, innerNeurons, nOfOutputs);
      if (nOfWeights != values.size()) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of values for weights: %d expected, %d found",
            nOfWeights,
            values.size()
        ));
      }
      return new QuantizedMultilayerSpikingNetwork(
          nOfInputs,
          innerNeurons,
          nOfOutputs,
          values.stream().mapToDouble(d -> d).toArray(),
          neuronBuilder
      );
    };
  }

  @Override
  public List<Double> exampleFor(QuantizedMultivariateSpikingFunction function) {
    return Collections.nCopies(
        QuantizedMultilayerSpikingNetwork.countWeights(
            MultiLayerPerceptron.countNeurons(
                function.getInputDimension(),
                innerNeurons(function.getInputDimension(), function.getOutputDimension()),
                function.getOutputDimension())
        ),
        0d
    );
  }

}
