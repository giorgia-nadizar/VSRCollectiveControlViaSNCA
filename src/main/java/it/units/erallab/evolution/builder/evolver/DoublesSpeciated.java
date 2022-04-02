package it.units.erallab.evolution.builder.evolver;

import com.google.common.collect.Range;
import it.units.erallab.evolution.builder.PrototypedFunctionBuilder;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.util.Domain;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.speciation.KMeansSpeciator;
import it.units.malelab.jgea.core.evolver.speciation.SpeciatedEvolver;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.distance.LNorm;
import it.units.malelab.jgea.representation.sequence.FixedLengthListFactory;
import it.units.malelab.jgea.representation.sequence.numeric.GaussianMutation;
import it.units.malelab.jgea.representation.sequence.numeric.GeometricCrossover;
import it.units.malelab.jgea.representation.sequence.numeric.UniformDoubleFactory;

import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.SortedMap;
import java.util.function.Function;

/**
 * @author eric
 */
public class DoublesSpeciated implements EvolverBuilder<List<Double>> {

  private final static int SPECTRUM_SIZE = 8;
  private final static double SPECTRUM_MIN_FREQ = 0d;
  private final static double SPECTRUM_MAX_FREQ = 4d;

  public enum SpeciationCriterion {GENOTYPE, POSTURE, CENTER, FOOTPRINTS}

  private final int nPop;
  private final int nSpecies;
  private final double xOverProb;
  private final SpeciationCriterion criterion;

  public DoublesSpeciated(int nPop, int nSpecies, double xOverProb, SpeciationCriterion criterion) {
    this.nPop = nPop;
    this.nSpecies = nSpecies;
    this.xOverProb = xOverProb;
    this.criterion = criterion;
  }

  @Override
  public <T, F> Evolver<List<Double>, T, F> build(PrototypedFunctionBuilder<List<Double>, T> builder, T target, PartialComparator<F> comparator) {
    Function<Individual<List<Double>, T, F>, double[]> converter = switch (criterion) {
      case GENOTYPE -> i -> i.getGenotype().stream().mapToDouble(Double::doubleValue).toArray();
      case POSTURE -> i -> {
        if (i.getFitness() instanceof Outcome) {
          Outcome o = (Outcome) i.getFitness();
          return o.getAveragePosture(8).values().stream().mapToDouble(b -> b ? 1d : 0d).toArray();
        }
        throw new IllegalStateException(String.format("Cannot obtain double[] from %s: Outcome expected", i.getFitness().getClass().getSimpleName()));
      };
      case CENTER -> i -> {
        if (i.getFitness() instanceof Outcome) {
          Outcome o = (Outcome) i.getFitness();
          double[] xSpectrum = o.getCenterXPositionSpectrum(SPECTRUM_MIN_FREQ, SPECTRUM_MAX_FREQ, SPECTRUM_SIZE).values().stream()
              .mapToDouble(d -> d)
              .toArray();
          double[] ySpectrum = o.getCenterYPositionSpectrum(SPECTRUM_MIN_FREQ, SPECTRUM_MAX_FREQ, SPECTRUM_SIZE).values().stream()
              .mapToDouble(d -> d)
              .toArray();
          double[] angleSpectrum = o.getCenterAngleSpectrum(SPECTRUM_MIN_FREQ, SPECTRUM_MAX_FREQ, SPECTRUM_SIZE).values().stream()
              .mapToDouble(d -> d)
              .toArray();
          double[] spectrum = new double[SPECTRUM_SIZE * 3];
          System.arraycopy(xSpectrum, 0, spectrum, 0, SPECTRUM_SIZE);
          System.arraycopy(ySpectrum, 0, spectrum, SPECTRUM_SIZE, SPECTRUM_SIZE);
          System.arraycopy(angleSpectrum, 0, spectrum, 2 * SPECTRUM_SIZE, SPECTRUM_SIZE);
          return spectrum;
        }
        throw new IllegalStateException(String.format("Cannot obtain double[] from %s: Outcome expected", i.getFitness().getClass().getSimpleName()));
      };
      case FOOTPRINTS -> i -> {
        if (i.getFitness() instanceof Outcome) {
          Outcome o = (Outcome) i.getFitness();
          List<SortedMap<Domain, Double>> footprintsSpectra = o.getFootprintsSpectra(4, SPECTRUM_MIN_FREQ, SPECTRUM_MAX_FREQ, SPECTRUM_SIZE);
          return footprintsSpectra.stream()
              .map(SortedMap::values)
              .flatMap(Collection::stream)
              .mapToDouble(d -> d)
              .toArray();
        }
        throw new IllegalStateException(String.format("Cannot obtain double[] from %s: Outcome expected", i.getFitness().getClass().getSimpleName()));
      };
    };
    int length = builder.exampleFor(target).size();
    return new SpeciatedEvolver<>(
        builder.buildFor(target),
        new FixedLengthListFactory<>(length, new UniformDoubleFactory(-1d, 1d)),
        comparator.comparing(Individual::getFitness),
        nPop,
        Map.of(
            new GaussianMutation(.35d), 1d - xOverProb,
            new GeometricCrossover(Range.closed(-.5d, 1.5d)).andThen(new GaussianMutation(.1d)), xOverProb
        ),
        nPop / nSpecies,
        new KMeansSpeciator<>(
            nSpecies,
            -1,
            new LNorm(2),
            converter
        ),
        0.75d,
        true
    );
  }

}