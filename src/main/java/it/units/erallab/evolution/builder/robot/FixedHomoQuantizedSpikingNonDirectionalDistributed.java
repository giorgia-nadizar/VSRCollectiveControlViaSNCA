package it.units.erallab.evolution.builder.robot;

import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.core.controllers.DistributedSensingNonDirectional;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedDistributedSpikingSensing;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedLIFNeuron;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedMultivariateSpikingFunction;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.converters.stv.QuantizedSpikeTrainToValueConverter;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.converters.vts.QuantizedValueToSpikeTrainConverter;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.core.objects.SensingVoxel;
import it.units.erallab.hmsrobots.util.Grid;
import it.units.erallab.hmsrobots.util.SerializationUtils;

import java.util.List;
import java.util.Objects;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * @author eric
 */
public class FixedHomoQuantizedSpikingNonDirectionalDistributed implements PrototypedFunctionBuilder<QuantizedMultivariateSpikingFunction, Robot<? extends SensingVoxel>> {
  private final int signals;
  private final QuantizedValueToSpikeTrainConverter valueToSpikeTrainConverter;
  private final QuantizedSpikeTrainToValueConverter spikeTrainToValueConverter;

  public FixedHomoQuantizedSpikingNonDirectionalDistributed(int signals, QuantizedValueToSpikeTrainConverter valueToSpikeTrainConverter, QuantizedSpikeTrainToValueConverter spikeTrainToValueConverter) {
    this.signals = signals;
    this.valueToSpikeTrainConverter = valueToSpikeTrainConverter;
    this.spikeTrainToValueConverter = spikeTrainToValueConverter;
  }

  @Override
  public Function<QuantizedMultivariateSpikingFunction, Robot<? extends SensingVoxel>> buildFor(Robot<? extends SensingVoxel> robot) {
    int[] dim = getIODim(robot);
    Grid<? extends SensingVoxel> body = robot.getVoxels();
    int nOfInputs = dim[0];
    int nOfOutputs = dim[1];
    return function -> {
      if (function.getInputDimension() != nOfInputs) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of function input args: %d expected, %d found",
            nOfInputs,
            function.getInputDimension()
        ));
      }
      if (function.getOutputDimension() != nOfOutputs) {
        throw new IllegalArgumentException(String.format(
            "Wrong number of function output args: %d expected, %d found",
            nOfOutputs,
            function.getOutputDimension()
        ));
      }
      QuantizedDistributedSpikingSensing controller = new QuantizedDistributedSpikingSensing(body, signals, new QuantizedLIFNeuron(), valueToSpikeTrainConverter, spikeTrainToValueConverter);
      for (Grid.Entry<? extends SensingVoxel> entry : body) {
        if (entry.getValue() != null) {
          controller.getFunctions().set(entry.getX(), entry.getY(), SerializationUtils.clone(function));
        }
      }
      return new Robot<>(
          controller,
          SerializationUtils.clone(body)
      );
    };
  }

  @Override
  public QuantizedMultivariateSpikingFunction exampleFor(Robot<? extends SensingVoxel> robot) {
    int[] dim = getIODim(robot);
    return QuantizedMultivariateSpikingFunction.build(d -> d, dim[0], dim[1]);
  }

  private int[] getIODim(Robot<? extends SensingVoxel> robot) {
    Grid<? extends SensingVoxel> body = robot.getVoxels();
    SensingVoxel voxel = body.values().stream().filter(Objects::nonNull).findFirst().orElse(null);
    if (voxel == null) {
      throw new IllegalArgumentException("Target robot has no voxels");
    }
    int nOfInputs = DistributedSensingNonDirectional.nOfInputs(voxel, signals);
    int nOfOutputs = DistributedSensingNonDirectional.nOfOutputs(voxel, signals);
    List<Grid.Entry<? extends SensingVoxel>> wrongVoxels = body.stream()
        .filter(e -> e.getValue() != null)
        .filter(e -> DistributedSensingNonDirectional.nOfInputs(e.getValue(), signals) != nOfInputs)
        .collect(Collectors.toList());
    if (!wrongVoxels.isEmpty()) {
      throw new IllegalArgumentException(String.format(
          "Cannot build %s robot mapper for this body: all voxels should have %d inputs, but voxels at positions %s have %s",
          getClass().getSimpleName(),
          nOfInputs,
          wrongVoxels.stream()
              .map(e -> String.format("(%d,%d)", e.getX(), e.getY()))
              .collect(Collectors.joining(",")),
          wrongVoxels.stream()
              .map(e -> String.format("%d", DistributedSensingNonDirectional.nOfInputs(e.getValue(), signals)))
              .collect(Collectors.joining(","))
      ));
    }
    return new int[]{nOfInputs, nOfOutputs};
  }

}
