package it.units.erallab.evolution.builder.phenotype;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.TimedRealFunction;

import java.util.Collections;
import java.util.List;
import java.util.function.Function;

/**
 * @author eric
 */
public class MLP implements PrototypedFunctionBuilder<List<Double>, TimedRealFunction> {

  private final double innerLayerRatio;
  private final int nOfInnerLayers;
  private final MultiLayerPerceptron.ActivationFunction activationFunction;

  public MLP(double innerLayerRatio, int nOfInnerLayers) {
    this(innerLayerRatio, nOfInnerLayers, MultiLayerPerceptron.ActivationFunction.TANH);
  }

  public MLP(double innerLayerRatio, int nOfInnerLayers, MultiLayerPerceptron.ActivationFunction activationFunction) {
    this.innerLayerRatio = innerLayerRatio;
    this.nOfInnerLayers = nOfInnerLayers;
    this.activationFunction = activationFunction;
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
  public Function<List<Double>, TimedRealFunction> buildFor(TimedRealFunction function) {
    return values -> {
      int nOfInputs = function.getInputDimension();
      int nOfOutputs = function.getOutputDimension();
      int[] innerNeurons = innerNeurons(nOfInputs, nOfOutputs);
      int nOfWeights = MultiLayerPerceptron.countWeights(nOfInputs, innerNeurons, nOfOutputs);
      if (nOfWeights != values.size()) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of values for weights: %d expected, %d found",
            nOfWeights,
            values.size()
        ));
      }
      return new MultiLayerPerceptron(
          activationFunction,
          nOfInputs,
          innerNeurons,
          nOfOutputs,
          values.stream().mapToDouble(d -> d).toArray()
      );
    };
  }

  @Override
  public List<Double> exampleFor(TimedRealFunction function) {
    return Collections.nCopies(
        MultiLayerPerceptron.countWeights(
            MultiLayerPerceptron.countNeurons(
                function.getInputDimension(),
                innerNeurons(function.getInputDimension(), function.getOutputDimension()),
                function.getOutputDimension())
        ),
        0d
    );
  }

}
