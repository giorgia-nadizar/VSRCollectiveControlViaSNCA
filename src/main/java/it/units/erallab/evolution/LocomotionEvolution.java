/*
 * Copyright 2020 Eric Medvet <eric.medvet@gmail.com> (as eric)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package it.units.erallab.evolution;

import com.google.common.base.Stopwatch;
import com.google.common.collect.Lists;
import it.units.erallab.evolution.builder.*;
import it.units.erallab.evolution.builder.evolver.*;
import it.units.erallab.evolution.builder.phenotype.*;
import it.units.erallab.evolution.builder.robot.*;
import it.units.erallab.evolution.utils.Utils;
import it.units.erallab.hmsrobots.core.controllers.Controller;
import it.units.erallab.hmsrobots.core.controllers.MultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.PruningMultiLayerPerceptron;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedIzhikevicNeuron;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedLIFNeuron;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedLIFNeuronWithHomeostasis;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.QuantizedSpikingFunction;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.converters.stv.QuantizedAverageFrequencySpikeTrainToValueConverter;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.converters.stv.QuantizedMovingAverageSpikeTrainToValueConverter;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.converters.stv.QuantizedSpikeTrainToValueConverter;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.converters.vts.QuantizedUniformValueToSpikeTrainConverter;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.converters.vts.QuantizedUniformWithMemoryValueToSpikeTrainConverter;
import it.units.erallab.hmsrobots.core.controllers.snndiscr.converters.vts.QuantizedValueToSpikeTrainConverter;
import it.units.erallab.hmsrobots.core.objects.Robot;
import it.units.erallab.hmsrobots.tasks.locomotion.Locomotion;
import it.units.erallab.hmsrobots.tasks.locomotion.Outcome;
import it.units.erallab.hmsrobots.util.RobotUtils;
import it.units.erallab.hmsrobots.viewers.drawers.Drawer;
import it.units.erallab.hmsrobots.viewers.drawers.Drawers;
import it.units.malelab.jgea.Worker;
import it.units.malelab.jgea.core.Individual;
import it.units.malelab.jgea.core.evolver.Event;
import it.units.malelab.jgea.core.evolver.Evolver;
import it.units.malelab.jgea.core.evolver.stopcondition.FitnessEvaluations;
import it.units.malelab.jgea.core.listener.*;
import it.units.malelab.jgea.core.listener.telegram.TelegramProgressMonitor;
import it.units.malelab.jgea.core.listener.telegram.TelegramUpdater;
import it.units.malelab.jgea.core.order.PartialComparator;
import it.units.malelab.jgea.core.util.Misc;
import it.units.malelab.jgea.core.util.Pair;
import it.units.malelab.jgea.core.util.SequentialFunction;
import org.dyn4j.dynamics.Settings;

import java.io.File;
import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.function.BiFunction;
import java.util.function.Function;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

import static it.units.erallab.hmsrobots.util.Utils.params;
import static it.units.malelab.jgea.core.listener.NamedFunctions.*;
import static it.units.malelab.jgea.core.util.Args.*;

/**
 * @author eric
 */
public class LocomotionEvolution extends Worker {

  private final static Settings PHYSICS_SETTINGS = new Settings();

  public static class ValidationOutcome {
    private final Event<?, ? extends Robot<?>, ? extends Outcome> event;
    private final Map<String, Object> keys;
    private final Outcome outcome;

    public ValidationOutcome(Event<?, ? extends Robot<?>, ? extends Outcome> event, Map<String, Object> keys, Outcome outcome) {
      this.event = event;
      this.keys = keys;
      this.outcome = outcome;
    }
  }

  public static final int CACHE_SIZE = 1000;
  public static final String MAPPER_PIPE_CHAR = "<";
  public static final String SEQUENCE_SEPARATOR_CHAR = ">";
  public static final String SEQUENCE_ITERATION_CHAR = ":";

  public LocomotionEvolution(String[] args) {
    super(args);
  }

  public static void main(String[] args) {
    new LocomotionEvolution(args);
  }

  @Override
  public void run() {
    int spectrumSize = 10;
    double spectrumMinFreq = 0d;
    double spectrumMaxFreq = 5d;
    double episodeTime = d(a("episodeTime", "10"));
    double episodeTransientTime = d(a("episodeTransientTime", "1"));
    double validationEpisodeTime = d(a("validationEpisodeTime", Double.toString(episodeTime)));
    double validationEpisodeTransientTime = d(a("validationEpisodeTransientTime", Double.toString(episodeTransientTime)));
    double videoEpisodeTime = d(a("videoEpisodeTime", "10"));
    double videoEpisodeTransientTime = d(a("videoEpisodeTransientTime", "0"));
    int nEvals = i(a("nEvals", "1000"));
    int[] seeds = ri(a("seed", "0:1"));
    String experimentName = a("expName", "short");
    List<String> terrainNames = l(a("terrain", "hilly-1-10-rnd"));
    List<String> targetShapeNames = l(a("shape", "biped-4x3"));
    List<String> targetSensorConfigNames = l(a("sensorConfig", "spinedTouchSighted-f-f-0.01"));
    List<String> transformationNames = l(a("transformation", "identity"));
    List<String> evolverNames = l(a("evolver", "ES-40-0.35-f"));
    List<String> mapperNames = l(a("mapper", "fixedCentralized<MLP-1-1-tanh"));
    String bestFileName = a("bestFile", null);
    String lastFileName = a("lastFile", null);
    String allFileName = a("allFile", null);
    String finalFileName = a("finalFile", null);
    String validationFileName = a("validationFile", null);
    boolean deferred = a("deferred", "true").startsWith("t");
    String telegramBotId = a("telegramBotId", null);
    long telegramChatId = Long.parseLong(a("telegramChatId", "0"));
    List<String> serializationFlags = l(a("serialization", "")); //last,best,all,final
    List<String> genotypeSerializationFlags = l(a("genoSerialization", "")); //last,best,all,final
    boolean output = a("output", "false").startsWith("t");
    List<String> validationTransformationNames = l(a("validationTransformation", "")).stream().filter(s -> !s.isEmpty()).collect(Collectors.toList());
    List<String> validationTerrainNames = l(a("validationTerrain", "flat")).stream().filter(s -> !s.isEmpty()).collect(Collectors.toList());
    List<String> fitnessMetrics = l(a("fitness", "velocity"));
    String videoConfiguration = a("videoConfiguration", "basicWithMiniWorldAndBrain");
    boolean detailedOutcome = a("detailedOutcome", "false").startsWith("t");
    boolean cacheOutcome = a("cache", "false").startsWith("t");
    Pair<Pair<Integer, Integer>, Function<String, Drawer>> drawerSupplier = getDrawerSupplierFromName(videoConfiguration);
    Function<Outcome, Double> fitnessFunction = getFitnessFunctionFromName(fitnessMetrics.get(0));
    Function<Outcome, Double>[] fitnessFunctions = getFitnessFunctionsFromName(fitnessMetrics);
    //consumers
    List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> keysFunctions = Utils.keysFunctions();
    List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> basicFunctions = Utils.basicFunctions();
    List<NamedFunction<Individual<?, ? extends Robot<?>, ? extends Outcome>, ?>> basicIndividualFunctions = Utils.individualFunctions(fitnessFunction);
    List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> populationFunctions = Utils.populationFunctions(fitnessFunction);
    List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> computationTimeFunctions = Utils.computationTimeFunctions();
    List<NamedFunction<Event<?, ? extends Robot<?>, ? extends Outcome>, ?>> visualFunctions = Utils.visualFunctions(fitnessFunction);
    List<NamedFunction<Outcome, ?>> basicOutcomeFunctions = Utils.basicOutcomeFunctions();
    List<NamedFunction<Outcome, ?>> detailedOutcomeFunctions = Utils.detailedOutcomeFunctions(spectrumMinFreq, spectrumMaxFreq, spectrumSize);
    List<NamedFunction<Outcome, ?>> visualOutcomeFunctions = Utils.visualOutcomeFunctions(spectrumMinFreq, spectrumMaxFreq);
    Listener.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>> factory = Listener.Factory.deaf();
    ProgressMonitor progressMonitor = new ScreenProgressMonitor(System.out);
    if (bestFileName == null || output) {
      factory = factory.and(new TabularPrinter<>(Misc.concat(List.of(
          basicFunctions,
          populationFunctions,
          visualFunctions,
          computationTimeFunctions,
          NamedFunction.then(best(), basicIndividualFunctions),
          NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), basicOutcomeFunctions),
          NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), visualOutcomeFunctions)
      ))));
    }
    if (lastFileName != null) {
      factory = factory.and(new CSVPrinter<>(Misc.concat(List.of(
          keysFunctions,
          basicFunctions,
          populationFunctions,
          NamedFunction.then(best(), basicIndividualFunctions),
          NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), basicOutcomeFunctions),
          detailedOutcome ? NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), detailedOutcomeFunctions) : List.of(),
          NamedFunction.then(best(), Utils.serializationFunction(serializationFlags.contains("last"), solution())),
          NamedFunction.then(best(), Utils.serializationFunction(genotypeSerializationFlags.contains("last"), genotype()))
      )), new File(lastFileName)
      ).onLast());
    }
    if (bestFileName != null) {
      factory = factory.and(new CSVPrinter<>(Misc.concat(List.of(
          keysFunctions,
          basicFunctions,
          populationFunctions,
          NamedFunction.then(best(), basicIndividualFunctions),
          NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), basicOutcomeFunctions),
          detailedOutcome ? NamedFunction.then(as(Outcome.class).of(fitness()).of(best()), detailedOutcomeFunctions) : List.of(),
          NamedFunction.then(best(), Utils.serializationFunction(serializationFlags.contains("best"), solution())),
          NamedFunction.then(best(), Utils.serializationFunction(genotypeSerializationFlags.contains("best"), genotype()))
      )), new File(bestFileName)
      ));
    }
    if (allFileName != null) {
      factory = factory.and(Listener.Factory.forEach(
          event -> event.getOrderedPopulation().all().stream()
              .map(i -> Pair.of(event, i))
              .collect(Collectors.toList()),
          new CSVPrinter<>(
              Misc.concat(List.of(
                  NamedFunction.then(f("event", Pair::first), keysFunctions),
                  NamedFunction.then(f("event", Pair::first), basicFunctions),
                  NamedFunction.then(f("individual", Pair::second), basicIndividualFunctions),
                  NamedFunction.then(f("individual", Pair::second), Utils.serializationFunction(serializationFlags.contains("all"), solution())),
                  NamedFunction.then(f("individual", Pair::second), Utils.serializationFunction(genotypeSerializationFlags.contains("all"), genotype()))
              )),
              new File(allFileName)
          )
      ));
    }
    if (finalFileName != null) {
      Listener.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>> entirePopulationFactory = Listener.Factory.forEach(
          event -> event.getOrderedPopulation().all().stream()
              .map(i -> Pair.of(event, i))
              .collect(Collectors.toList()),
          new CSVPrinter<>(
              Misc.concat(List.of(
                  NamedFunction.then(f("event", Pair::first), keysFunctions),
                  NamedFunction.then(f("event", Pair::first), basicFunctions),
                  NamedFunction.then(f("individual", Pair::second), basicIndividualFunctions),
                  NamedFunction.then(f("individual", Pair::second), Utils.serializationFunction(serializationFlags.contains("final"), solution())),
                  NamedFunction.then(f("individual", Pair::second), Utils.serializationFunction(genotypeSerializationFlags.contains("final"), genotype()))
              )),
              new File(finalFileName)
          )
      );
      factory = factory.and(entirePopulationFactory.onLast());
    }
    //validation listener
    if (validationFileName != null) {
      if (!validationTerrainNames.isEmpty() && validationTransformationNames.isEmpty()) {
        validationTransformationNames.add("identity");
      }
      if (validationTerrainNames.isEmpty() && !validationTransformationNames.isEmpty()) {
        validationTerrainNames.add(terrainNames.get(0));
      }
      Listener.Factory<Event<?, ? extends Robot<?>, ? extends Outcome>> validationFactory = Listener.Factory.forEach(
          Utils.validation(validationTerrainNames, validationTransformationNames, List.of(0), validationEpisodeTime),
          new CSVPrinter<>(
              Misc.concat(List.of(
                  NamedFunction.then(f("event", (ValidationOutcome vo) -> vo.event), basicFunctions),
                  NamedFunction.then(f("event", (ValidationOutcome vo) -> vo.event), keysFunctions),
                  NamedFunction.then(f("keys", (ValidationOutcome vo) -> vo.keys), List.of(
                      f("validation.terrain", (Map<String, Object> map) -> map.get("validation.terrain")),
                      f("validation.transformation", (Map<String, Object> map) -> map.get("validation.transformation")),
                      f("validation.seed", "%2d", (Map<String, Object> map) -> map.get("validation.seed"))
                  )),
                  NamedFunction.then(
                      f("outcome", (ValidationOutcome vo) -> vo.outcome.subOutcome(validationEpisodeTransientTime, validationEpisodeTime)),
                      basicOutcomeFunctions
                  ),
                  detailedOutcome ?
                      NamedFunction.then(
                          f("outcome", (ValidationOutcome vo) -> vo.outcome.subOutcome(validationEpisodeTransientTime, validationEpisodeTime)),
                          detailedOutcomeFunctions
                      ) : List.of()
              )),
              new File(validationFileName)
          )
      ).onLast();
      factory = factory.and(validationFactory);
    }
    if (telegramBotId != null && telegramChatId != 0) {
      factory = factory.and(new TelegramUpdater<>(List.of(
          Utils.lastEventToString(fitnessFunction),
          Utils.fitnessPlot(fitnessFunction),
          Utils.centerPositionPlot(),
          Utils.bestVideo(videoEpisodeTransientTime, videoEpisodeTime, PHYSICS_SETTINGS, drawerSupplier)
      ), telegramBotId, telegramChatId));
      progressMonitor = progressMonitor.and(new TelegramProgressMonitor(telegramBotId, telegramChatId));
    }
    //summarize params
    L.info("Experiment name: " + experimentName);
    L.info("N evaluations: " + nEvals);
    L.info("Evolvers: " + evolverNames);
    L.info("Mappers: " + mapperNames);
    L.info("Fitness metrics: " + fitnessMetrics);
    L.info("Shapes: " + targetShapeNames);
    L.info("Sensor configs: " + targetSensorConfigNames);
    L.info("Terrains: " + terrainNames);
    L.info("Transformations: " + transformationNames);
    L.info("Validations: " + Lists.cartesianProduct(validationTerrainNames, validationTransformationNames));
    //start iterations
    int nOfRuns = seeds.length * terrainNames.size() * targetShapeNames.size() * targetSensorConfigNames.size() * mapperNames.size() * transformationNames.size() * evolverNames.size();
    int counter = 0;
    for (int seed : seeds) {
      for (String terrainName : terrainNames) {
        for (String targetShapeName : targetShapeNames) {
          for (String targetSensorConfigName : targetSensorConfigNames) {
            for (String mapperName : mapperNames) {
              for (String transformationName : transformationNames) {
                for (String evolverName : evolverNames) {
                  counter = counter + 1;
                  final Random random = new Random(seed);
                  //prepare keys
                  Map<String, Object> keys = Map.ofEntries(
                      Map.entry("experiment.name", experimentName),
                      Map.entry("seed", seed),
                      Map.entry("terrain", terrainName),
                      Map.entry("shape", targetShapeName),
                      Map.entry("sensor.config", targetSensorConfigName),
                      Map.entry("mapper", mapperName),
                      Map.entry("transformation", transformationName),
                      Map.entry("evolver", evolverName),
                      Map.entry("fitness.metrics", fitnessMetrics)
                  );
                  Robot<?> target = new Robot<>(
                      Controller.empty(),
                      RobotUtils.buildSensorizingFunction(targetSensorConfigName).apply(RobotUtils.buildShape(targetShapeName))
                  );
                  //build evolver
                  Evolver<?, Robot<?>, Outcome> evolver;
                  try {
                    evolver = buildEvolver(evolverName, mapperName, target, fitnessFunctions);
                  } catch (ClassCastException | IllegalArgumentException e) {
                    e.printStackTrace();
                    L.warning(String.format(
                        "Cannot instantiate %s for %s: %s",
                        evolverName,
                        mapperName,
                        e
                    ));
                    continue;
                  }
                  Listener<Event<?, ? extends Robot<?>, ? extends Outcome>> listener = Listener.all(List.of(
                      new EventAugmenter(keys),
                      factory.build()
                  ));
                  if (deferred) {
                    listener = listener.deferred(executorService);
                  }
                  //optimize
                  Stopwatch stopwatch = Stopwatch.createStarted();
                  progressMonitor.notify(((float) counter - 1) / nOfRuns, String.format("(%d/%d); Starting %s", counter, nOfRuns, keys));
                  //build task
                  try {
                    Collection<Robot<?>> solutions = evolver.solve(
                        buildTaskFromName(transformationName, terrainName, episodeTime, random, cacheOutcome)
                            .andThen(o -> o.subOutcome(episodeTransientTime, episodeTime)),
                        new FitnessEvaluations(nEvals),
                        random,
                        executorService,
                        listener
                    );
                    progressMonitor.notify((float) counter / nOfRuns, String.format("(%d/%d); Done: %d solutions in %4ds", counter, nOfRuns, solutions.size(), stopwatch.elapsed(TimeUnit.SECONDS)));
                  } catch (Exception e) {
                    L.severe(String.format("Cannot complete %s due to %s",
                        keys,
                        e
                    ));
                    e.printStackTrace(); // TODO possibly to be removed
                  }
                }
              }
            }
          }
        }
      }
    }
    factory.shutdown();
  }

  private static EvolverBuilder<?> getEvolverBuilderFromName(String name) {
    String numGA = "numGA-(?<nPop>\\d+)-(?<diversity>(t|f))-(?<remap>(t|f))";
    String numGASpeciated = "numGASpec-(?<nPop>\\d+)-(?<nSpecies>\\d+)-(?<criterion>(" + Arrays.stream(DoublesSpeciated.SpeciationCriterion.values()).map(c -> c.name().toLowerCase(Locale.ROOT)).collect(Collectors.joining("|")) + "))";
    String bitGA = "bitGA-(?<nPop>\\d+)-(?<diversity>(t|f))-(?<remap>(t|f))";
    String ternaryGA = "terGA-(?<nPop>\\d+)-(?<diversity>(t|f))-(?<remap>(t|f))";
    String cmaES = "CMAES";
    String eS = "ES-(?<nPop>\\d+)-(?<sigma>\\d+(\\.\\d+)?)-(?<remap>(t|f))";
    String bitNumGA = "bitNumGA-(?<nPop>\\d+)-(?<diversity>(t|f))-(?<remap>(t|f))";
    String biasedBitNumGA = "biasedBitNumGA-(?<nPop>\\d+)-(?<diversity>(t|f))-(?<remap>(t|f))";
    String bitNumMutation = "bitNumMut-(?<nPop>\\d+)-(?<diversity>(t|f))-(?<remap>(t|f))";
    Map<String, String> params;
    if ((params = params(numGA, name)) != null) {
      return new DoublesStandard(
          Integer.parseInt(params.get("nPop")),
          (int) Math.max(Math.round((double) Integer.parseInt(params.get("nPop")) / 10d), 3),
          0.75d,
          params.get("diversity").equals("t"),
          params.get("diversity").equals("remap")
      );
    }
    if ((params = params(bitGA, name)) != null) {
      return new BinaryStandard(
          Integer.parseInt(params.get("nPop")),
          (int) Math.max(Math.round((double) Integer.parseInt(params.get("nPop")) / 10d), 3),
          0.75d,
          params.get("diversity").equals("t"),
          params.get("diversity").equals("remap")
      );
    }
    if ((params = params(ternaryGA, name)) != null) {
      return new IntegersStandard(
          Integer.parseInt(params.get("nPop")),
          (int) Math.max(Math.round((double) Integer.parseInt(params.get("nPop")) / 10d), 3),
          0.75d,
          params.get("diversity").equals("t"),
          params.get("remap").equals("t")
      );
    }
    if ((params = params(bitNumGA, name)) != null) {
      return new BinaryAndDoublesStandard(
          Integer.parseInt(params.get("nPop")),
          (int) Math.max(Math.round((double) Integer.parseInt(params.get("nPop")) / 10d), 3),
          0.75d,
          params.get("diversity").equals("t"),
          params.get("remap").equals("t")
      );
    }
    if ((params = params(biasedBitNumGA, name)) != null) {
      return new BinaryAndDoublesBiased(
          Integer.parseInt(params.get("nPop")),
          (int) Math.max(Math.round((double) Integer.parseInt(params.get("nPop")) / 10d), 3),
          0d,
          params.get("diversity").equals("t"),
          params.get("remap").equals("t")
      );
    }
    if ((params = params(bitNumMutation, name)) != null) {
      return new BinaryAndDoublesStandard(
          Integer.parseInt(params.get("nPop")),
          (int) Math.max(Math.round((double) Integer.parseInt(params.get("nPop")) / 10d), 3),
          0d,
          params.get("diversity").equals("t"),
          params.get("remap").equals("t")
      );
    }
    if ((params = params(numGASpeciated, name)) != null) {
      return new DoublesSpeciated(
          Integer.parseInt(params.get("nPop")),
          Integer.parseInt(params.get("nSpecies")),
          0.75d,
          DoublesSpeciated.SpeciationCriterion.valueOf(params.get("criterion").toUpperCase())
      );
    }
    if ((params = params(eS, name)) != null) {
      return new ES(
          Double.parseDouble(params.get("sigma")),
          Integer.parseInt(params.get("nPop")),
          params.get("remap").equals("t")
      );
    }
    //noinspection UnusedAssignment
    if ((params = params(cmaES, name)) != null) {
      return new CMAES();
    }
    throw new IllegalArgumentException(String.format("Unknown evolver builder name: %s", name));
  }

  private static Function<Outcome, Double> getFitnessFunctionFromName(String name) {
    String velocity = "velocity";
    String roundedVelocity = "rounded-velocity";
    String efficiency = "efficiency";
    String avgSumOfAbsWeights = "avg-sum-abs-weights";
    if (params(velocity, name) != null) {
      return Outcome::getVelocity;
    }
    if (params(roundedVelocity, name) != null) {
      return o -> (double) Math.round(o.getVelocity());
    }
    if (params(efficiency, name) != null) {
      return Outcome::getCorrectedEfficiency;
    }
    if (params(avgSumOfAbsWeights, name) != null) {
      return Outcome::getAverageSumOfAbsoluteWeights;
    }
    throw new IllegalArgumentException(String.format("Unknown fitness function name: %s", name));
  }

  @SuppressWarnings({"unchecked", "rawtypes"})
  private static Function<Outcome, Double>[] getFitnessFunctionsFromName(List<String> names) {
    Function[] fitnessMeasures = new Function[names.size()];
    IntStream.range(0, names.size()).forEach(i -> fitnessMeasures[i] = getFitnessFunctionFromName(names.get(i)));
    return fitnessMeasures;
  }

  private static Pair<Pair<Integer, Integer>, Function<String, Drawer>> getDrawerSupplierFromName(String name) {
    String basic = "basic";
    String basicWithMiniWorld = "basicWithMiniWorld";
    String basicWithMiniWorldAndBrain = "basicWithMiniWorldAndBrain";
    String basicWithMiniWorldAndBrainUsage = "basicWithMiniWorldAndBrainUsage";
    if (params(basic, name) != null) {
      return Pair.of(Pair.of(1, 1), Drawers::basic);
    }
    if (params(basicWithMiniWorld, name) != null) {
      return Pair.of(Pair.of(1, 1), Drawers::basicWithMiniWorld);
    }
    if (params(basicWithMiniWorldAndBrain, name) != null) {
      return Pair.of(Pair.of(1, 2), Drawers::basicWithMiniWorldAndBrain);
    }
    if (params(basicWithMiniWorldAndBrainUsage, name) != null) {
      return Pair.of(Pair.of(1, 2), Drawers::basicWithMiniWorldAndBrainUsage);
    }
    throw new IllegalArgumentException(String.format("Unknown video configuration name: %s", name));
  }

  @SuppressWarnings({"unchecked", "rawtypes"})
  private static PrototypedFunctionBuilder<?, ?> getMapperBuilderFromName(String name) {
    String binary = "binary-(?<value>\\d+(\\.\\d+)?)";
    String ternary = "ternary-(?<value>\\d+(\\.\\d+)?)";
    String fixedCentralized = "fixedCentralized";
    String fixedHomoDistributed = "fixedHomoDist-(?<nSignals>\\d+)";
    String fixedHomoNonDirDistributed = "fixedHomoNonDirDist-(?<nSignals>\\d+)";
    String fixedHeteroDistributed = "fixedHeteroDist-(?<nSignals>\\d+)";
       String fixedHomoQuantizedSpikingDistributed = "fixedHomoQuantSpikeDist-(?<nSignals>\\d+)" +
        "-(?<iConv>(unif|unif_mem))-(?<iFreq>\\d+(\\.\\d+)?)-(?<oConv>(avg|avg_mem))(-(?<oMem>\\d+))?-(?<oFreq>\\d+(\\.\\d+)?)";
    String fixedHomoQuantizedSpikingNonDirDistributed = "fixedHomoQuantSpikeNonDirDist-(?<nSignals>\\d+)" +
        "-(?<iConv>(unif|unif_mem))-(?<iFreq>\\d+(\\.\\d+)?)-(?<oConv>(avg|avg_mem))(-(?<oMem>\\d+))?-(?<oFreq>\\d+(\\.\\d+)?)";
    String fixedHeteroQuantizedSpikingDistributed = "fixedHeteroQuantSpikeDist-(?<nSignals>\\d+)" +
        "-(?<iConv>(unif|unif_mem))-(?<iFreq>\\d+(\\.\\d+)?)-(?<oConv>(avg|avg_mem))(-(?<oMem>\\d+))?-(?<oFreq>\\d+(\\.\\d+)?)";
    String fixedPhasesFunction = "fixedPhasesFunct-(?<f>\\d+)";
    String fixedPhases = "fixedPhases-(?<f>\\d+)";
    String fixedPhasesAndFrequencies = "fixedPhasesAndFrequencies";
    String bodySin = "bodySin-(?<fullness>\\d+(\\.\\d+)?)-(?<minF>\\d+(\\.\\d+)?)-(?<maxF>\\d+(\\.\\d+)?)";
    String bodyAndHomoDistributed = "bodyAndHomoDist-(?<fullness>\\d+(\\.\\d+)?)-(?<nSignals>\\d+)-(?<nLayers>\\d+)";
    String sensorAndBodyAndHomoDistributed = "sensorAndBodyAndHomoDist-(?<fullness>\\d+(\\.\\d+)?)-(?<nSignals>\\d+)-(?<nLayers>\\d+)-(?<position>(t|f))";
    String sensorCentralized = "sensorCentralized-(?<nLayers>\\d+)";
    String mlp = "MLP-(?<ratio>\\d+(\\.\\d+)?)-(?<nLayers>\\d+)(-(?<actFun>(sin|tanh|sigmoid|relu)))?";
    String pruningMlp = "pMLP-(?<ratio>\\d+(\\.\\d+)?)-(?<nLayers>\\d+)-(?<actFun>(sin|tanh|sigmoid|relu))-(?<pruningTime>\\d+(\\.\\d+)?)-(?<pruningRate>0(\\.\\d+)?)-(?<criterion>(weight|abs_signal_mean|random))";
    String quantizedMsn = "QMSNd-(?<ratio>\\d+(\\.\\d+)?)-(?<nLayers>\\d+)-(?<spikeType>(lif|iz|lif_h))" +
        "(-(?<lRestPot>-?\\d+(\\.\\d+)?)-(?<lThreshPot>-?\\d+(\\.\\d+)?)-(?<lambda>\\d+(\\.\\d+)?)(-(?<theta>\\d+(\\.\\d+)?))?)?" +
        "(-(?<izParams>(regular_spiking_params)))?";
    String quantizedMsnWithConverter = "QMSN-(?<ratio>\\d+(\\.\\d+)?)-(?<nLayers>\\d+)-(?<spikeType>(lif|iz|lif_h|lif_h_output|lif_h_io))" +
        "(-(?<lRestPot>-?\\d+(\\.\\d+)?)-(?<lThreshPot>-?\\d+(\\.\\d+)?)-(?<lambda>\\d+(\\.\\d+)?)(-(?<theta>\\d+(\\.\\d+)?))?)?" +
        "(-(?<izParams>(regular_spiking_params)))?" +
        "-(?<iConv>(unif|unif_mem))-(?<iFreq>\\d+(\\.\\d+)?)-(?<oConv>(avg|avg_mem))(-(?<oMem>\\d+))?-(?<oFreq>\\d+(\\.\\d+)?)";
    String oneHotMsnWithConverter = "HMSN-(?<ratio>\\d+(\\.\\d+)?)-(?<nLayers>\\d+)-(?<spikeType>(lif|iz|lif_h|lif_h_output|lif_h_io))" +
        "(-(?<lRestPot>-?\\d+(\\.\\d+)?)-(?<lThreshPot>-?\\d+(\\.\\d+)?)-(?<lambda>\\d+(\\.\\d+)?)(-(?<theta>\\d+(\\.\\d+)?))?)?" +
        "(-(?<izParams>(regular_spiking_params)))?";

    String directNumGrid = "directNumGrid";
    String functionNumGrid = "functionNumGrid";
    String fgraph = "fGraph";
    String functionGrid = "fGrid-(?<innerMapper>.*)";
    String spikingFunctionGrid = "snnFuncGrid-(?<innerMapper>.*)";
    String spikingQuantizedFunctionGrid = "snnQuantFuncGrid-(?<innerMapper>.*)";
    Map<String, String> params;
    //robot mappers
    if ((params = params(binary, name)) != null) {
      return new BinaryToDoubles(Double.parseDouble(params.get("value")));
    }
    if ((params = params(ternary, name)) != null) {
      return new TernaryToDoubles(Double.parseDouble(params.get("value")));
    }
    //noinspection UnusedAssignment
    if ((params = params(fixedCentralized, name)) != null) {
      return new FixedCentralized();
    }
    if ((params = params(fixedHomoDistributed, name)) != null) {
      return new FixedHomoDistributed(
          Integer.parseInt(params.get("nSignals"))
      );
    }
    if ((params = params(fixedHomoNonDirDistributed, name)) != null) {
      return new FixedHomoNonDirectionalDistributed(
          Integer.parseInt(params.get("nSignals"))
      );
    }
    if ((params = params(fixedHeteroDistributed, name)) != null) {
      return new FixedHeteroDistributed(
          Integer.parseInt(params.get("nSignals"))
      );
    }
    if ((params = params(fixedHomoQuantizedSpikingDistributed, name)) != null ||
        (params = params(fixedHomoQuantizedSpikingNonDirDistributed, name)) != null) {
      QuantizedValueToSpikeTrainConverter valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter();
      QuantizedSpikeTrainToValueConverter spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter();
      if (params.containsKey("iConv")) {
        switch (params.get("iConv")) {
          case "unif":
            if (params.containsKey("iFreq")) {
              valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter(Double.parseDouble(params.get("iFreq")));
            } else {
              valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter();
            }
            break;
          case "unif_mem":
            if (params.containsKey("iFreq")) {
              valueToSpikeTrainConverter = new QuantizedUniformWithMemoryValueToSpikeTrainConverter(Double.parseDouble(params.get("iFreq")));
            } else {
              valueToSpikeTrainConverter = new QuantizedUniformWithMemoryValueToSpikeTrainConverter();
            }
            break;
        }
      }
      if (params.containsKey("oConv")) {
        switch (params.get("oConv")) {
          case "avg":
            if (params.containsKey("oFreq")) {
              spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")));
            } else {
              spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter();
            }
            break;
          case "avg_mem":
            if (params.containsKey("oFreq")) {
              if (params.containsKey("oMem")) {
                spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")),
                    Integer.parseInt(params.get("oMem")));
              } else {
                spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")));
              }
            } else {
              if (params.containsKey("oMem")) {
                spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Integer.parseInt(params.get("oMem")));
              } else {
                spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter();
              }
            }
            break;
        }
      }
      if ((params = params(fixedHomoQuantizedSpikingDistributed, name)) != null) {
        return new FixedHomoQuantizedSpikingDistributed(
            Integer.parseInt(params.get("nSignals")),
            valueToSpikeTrainConverter,
            spikeTrainToValueConverter
        );
      } else {
        params = params(fixedHomoQuantizedSpikingNonDirDistributed, name);
        return new FixedHomoQuantizedSpikingNonDirectionalDistributed(
            Integer.parseInt(params.get("nSignals")),
            valueToSpikeTrainConverter,
            spikeTrainToValueConverter
        );
      }
    }
    if ((params = params(fixedHeteroQuantizedSpikingDistributed, name)) != null) {
      QuantizedValueToSpikeTrainConverter valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter();
      QuantizedSpikeTrainToValueConverter spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter();
      if (params.containsKey("iConv")) {
        switch (params.get("iConv")) {
          case "unif":
            if (params.containsKey("iFreq")) {
              valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter(Double.parseDouble(params.get("iFreq")));
            } else {
              valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter();
            }
            break;
          case "unif_mem":
            if (params.containsKey("iFreq")) {
              valueToSpikeTrainConverter = new QuantizedUniformWithMemoryValueToSpikeTrainConverter(Double.parseDouble(params.get("iFreq")));
            } else {
              valueToSpikeTrainConverter = new QuantizedUniformWithMemoryValueToSpikeTrainConverter();
            }
            break;
        }
      }
      if (params.containsKey("oConv")) {
        switch (params.get("oConv")) {
          case "avg":
            if (params.containsKey("oFreq")) {
              spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")));
            } else {
              spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter();
            }
            break;
          case "avg_mem":
            if (params.containsKey("oFreq")) {
              if (params.containsKey("oMem")) {
                spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")),
                    Integer.parseInt(params.get("oMem")));
              } else {
                spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")));
              }
            } else {
              if (params.containsKey("oMem")) {
                spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Integer.parseInt(params.get("oMem")));
              } else {
                spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter();
              }
            }
            break;
        }
      }
      return new FixedHeteroQuantizedSpikingDistributed(
          Integer.parseInt(params.get("nSignals")),
          valueToSpikeTrainConverter,
          spikeTrainToValueConverter
      );
    }
    if ((params = params(fixedPhasesFunction, name)) != null) {
      return new FixedPhaseFunction(
          Double.parseDouble(params.get("f")),
          1d
      );
    }
    if ((params = params(fixedPhases, name)) != null) {
      return new FixedPhaseValues(
          Double.parseDouble(params.get("f")),
          1d
      );
    }
    //noinspection UnusedAssignment
    if ((params = params(fixedPhasesAndFrequencies, name)) != null) {
      return new FixedPhaseAndFrequencyValues(
          1d
      );
    }
    if ((params = params(bodyAndHomoDistributed, name)) != null) {
      return new BodyAndHomoDistributed(
          Integer.parseInt(params.get("nSignals")),
          Double.parseDouble(params.get("fullness"))
      )
          .compose(PrototypedFunctionBuilder.of(List.of(
              new MLP(2d, 3, MultiLayerPerceptron.ActivationFunction.SIN),
              new MLP(0.65d, Integer.parseInt(params.get("nLayers")))
          )))
          .compose(PrototypedFunctionBuilder.merger());
    }
    if ((params = params(sensorAndBodyAndHomoDistributed, name)) != null) {
      return new SensorAndBodyAndHomoDistributed(
          Integer.parseInt(params.get("nSignals")),
          Double.parseDouble(params.get("fullness")),
          params.get("position").equals("t")
      )
          .compose(PrototypedFunctionBuilder.of(List.of(
              new MLP(2d, 3, MultiLayerPerceptron.ActivationFunction.SIN),
              new MLP(1.5d, Integer.parseInt(params.get("nLayers")))
          )))
          .compose(PrototypedFunctionBuilder.merger());
    }
    if ((params = params(bodySin, name)) != null) {
      return new BodyAndSinusoidal(
          Double.parseDouble(params.get("minF")),
          Double.parseDouble(params.get("maxF")),
          Double.parseDouble(params.get("fullness")),
          Set.of(BodyAndSinusoidal.Component.FREQUENCY, BodyAndSinusoidal.Component.PHASE, BodyAndSinusoidal.Component.AMPLITUDE)
      );
    }
    if ((params = params(sensorCentralized, name)) != null) {
      return new SensorCentralized()
          .compose(PrototypedFunctionBuilder.of(List.of(
              new MLP(2d, 3, MultiLayerPerceptron.ActivationFunction.SIN),
              new MLP(1.5d, Integer.parseInt(params.get("nLayers")))
          )))
          .compose(PrototypedFunctionBuilder.merger());
    }
    //function mappers
    if ((params = params(mlp, name)) != null) {
      return new MLP(
          Double.parseDouble(params.get("ratio")),
          Integer.parseInt(params.get("nLayers")),
          params.containsKey("actFun") ? MultiLayerPerceptron.ActivationFunction.valueOf(params.get("actFun").toUpperCase()) : MultiLayerPerceptron.ActivationFunction.TANH
      );
    }
    if ((params = params(pruningMlp, name)) != null) {
      return new PruningMLP(
          Double.parseDouble(params.get("ratio")),
          Integer.parseInt(params.get("nLayers")),
          MultiLayerPerceptron.ActivationFunction.valueOf(params.get("actFun").toUpperCase()),
          Double.parseDouble(params.get("pruningTime")),
          Double.parseDouble(params.get("pruningRate")),
          PruningMultiLayerPerceptron.Context.NETWORK,
          PruningMultiLayerPerceptron.Criterion.valueOf(params.get("criterion").toUpperCase())
      );
    }
    if ((params = params(quantizedMsn, name)) != null ||
        (params = params(oneHotMsnWithConverter, name)) != null ||
        (params = params(quantizedMsnWithConverter, name)) != null
    ) {
      BiFunction<Integer, Integer, QuantizedSpikingFunction> neuronBuilder = null;
      if (params.containsKey("spikeType") && params.get("spikeType").equals("lif")) {
        if (params.containsKey("lRestPot") && params.containsKey("lThreshPot") && params.containsKey("lambda")) {
          double restingPotential = Double.parseDouble(params.get("lRestPot"));
          double thresholdPotential = Double.parseDouble(params.get("lThreshPot"));
          double lambda = Double.parseDouble(params.get("lambda"));
          neuronBuilder = (l, n) -> new QuantizedLIFNeuron(restingPotential, thresholdPotential, lambda);
        } else {
          neuronBuilder = (l, n) -> new QuantizedLIFNeuron();
        }
      }
      if (params.containsKey("spikeType") && params.get("spikeType").equals("lif_h")) {
        if (params.containsKey("lRestPot") && params.containsKey("lThreshPot") && params.containsKey("lambda") && params.containsKey("theta")) {
          double restingPotential = Double.parseDouble(params.get("lRestPot"));
          double thresholdPotential = Double.parseDouble(params.get("lThreshPot"));
          double lambda = Double.parseDouble(params.get("lambda"));
          double theta = Double.parseDouble(params.get("theta"));
          neuronBuilder = (l, n) -> new QuantizedLIFNeuronWithHomeostasis(restingPotential, thresholdPotential, lambda, theta);
        } else {
          neuronBuilder = (l, n) -> new QuantizedLIFNeuronWithHomeostasis();
        }
      }
      if (params.containsKey("spikeType") && params.get("spikeType").equals("lif_h_output")) {
        int outputLayerIndex = Integer.parseInt(params.get("nLayers")) + 1;
        if (params.containsKey("lRestPot") && params.containsKey("lThreshPot") && params.containsKey("lambda") && params.containsKey("theta")) {
          double restingPotential = Double.parseDouble(params.get("lRestPot"));
          double thresholdPotential = Double.parseDouble(params.get("lThreshPot"));
          double lambda = Double.parseDouble(params.get("lambda"));
          double theta = Double.parseDouble(params.get("theta"));
          neuronBuilder = (l, n) -> (l == outputLayerIndex) ? new QuantizedLIFNeuronWithHomeostasis(restingPotential, thresholdPotential, lambda, theta) : new QuantizedLIFNeuron(restingPotential, thresholdPotential, lambda);
        } else {
          neuronBuilder = (l, n) -> (l == outputLayerIndex) ? new QuantizedLIFNeuronWithHomeostasis() : new QuantizedLIFNeuron();
        }
      }
      if (params.containsKey("spikeType") && params.get("spikeType").equals("lif_h_io")) {
        int outputLayerIndex = Integer.parseInt(params.get("nLayers")) + 1;
        if (params.containsKey("lRestPot") && params.containsKey("lThreshPot") && params.containsKey("lambda") && params.containsKey("theta")) {
          double restingPotential = Double.parseDouble(params.get("lRestPot"));
          double thresholdPotential = Double.parseDouble(params.get("lThreshPot"));
          double lambda = Double.parseDouble(params.get("lambda"));
          double theta = Double.parseDouble(params.get("theta"));
          neuronBuilder = (l, n) -> (l == outputLayerIndex || l == 0) ? new QuantizedLIFNeuronWithHomeostasis(restingPotential, thresholdPotential, lambda, theta) : new QuantizedLIFNeuron(restingPotential, thresholdPotential, lambda);
        } else {
          neuronBuilder = (l, n) -> (l == outputLayerIndex || l == 0) ? new QuantizedLIFNeuronWithHomeostasis() : new QuantizedLIFNeuron();
        }
      }
      if (params.containsKey("spikeType") && params.get("spikeType").equals("iz")) {
        if (params.containsKey("izParams")) {
          QuantizedIzhikevicNeuron.IzhikevicParameters izhikevicParameters = QuantizedIzhikevicNeuron.IzhikevicParameters.valueOf(params.get("izParams").toUpperCase());
          neuronBuilder = (l, n) -> new QuantizedIzhikevicNeuron(izhikevicParameters);
        } else {
          neuronBuilder = (l, n) -> new QuantizedIzhikevicNeuron();
        }
      }
      if ((params = params(quantizedMsn, name)) != null) {
        return new QuantizedMSN(
            Double.parseDouble(params.get("ratio")),
            Integer.parseInt(params.get("nLayers")),
            neuronBuilder
        );
      }
      if ((params = params(quantizedMsnWithConverter, name)) != null) {
        QuantizedValueToSpikeTrainConverter valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter();
        QuantizedSpikeTrainToValueConverter spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter();
        if (params.containsKey("iConv")) {
          switch (params.get("iConv")) {
            case "unif":
              if (params.containsKey("iFreq")) {
                valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter(Double.parseDouble(params.get("iFreq")));
              } else {
                valueToSpikeTrainConverter = new QuantizedUniformValueToSpikeTrainConverter();
              }
              break;
            case "unif_mem":
              if (params.containsKey("iFreq")) {
                valueToSpikeTrainConverter = new QuantizedUniformWithMemoryValueToSpikeTrainConverter(Double.parseDouble(params.get("iFreq")));
              } else {
                valueToSpikeTrainConverter = new QuantizedUniformWithMemoryValueToSpikeTrainConverter();
              }
              break;
          }
        }
        if (params.containsKey("oConv")) {
          switch (params.get("oConv")) {
            case "avg":
              if (params.containsKey("oFreq")) {
                spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")));
              } else {
                spikeTrainToValueConverter = new QuantizedAverageFrequencySpikeTrainToValueConverter();
              }
              break;
            case "avg_mem":
              if (params.containsKey("oFreq")) {
                if (params.containsKey("oMem")) {
                  spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")),
                      Integer.parseInt(params.get("oMem")));
                } else {
                  spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Double.parseDouble(params.get("oFreq")));
                }
              } else {
                if (params.containsKey("oMem")) {
                  spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter(Integer.parseInt(params.get("oMem")));
                } else {
                  spikeTrainToValueConverter = new QuantizedMovingAverageSpikeTrainToValueConverter();
                }
              }
              break;
          }
        }
        return new QuantizedMSNWithConverters(
            Double.parseDouble(params.get("ratio")),
            Integer.parseInt(params.get("nLayers")),
            neuronBuilder,
            valueToSpikeTrainConverter,
            spikeTrainToValueConverter
        );
      }
    }

    //noinspection UnusedAssignment
    if ((params = params(fgraph, name)) != null) {
      return new FGraph();
    }
    //misc
    if ((params = params(functionGrid, name)) != null) {
      return new FunctionGrid((PrototypedFunctionBuilder) getMapperBuilderFromName(params.get("innerMapper")));
    }
    if ((params = params(spikingFunctionGrid, name)) != null) {
      return new FunctionGrid((PrototypedFunctionBuilder) getMapperBuilderFromName(params.get("innerMapper")));
    }
    if ((params = params(spikingQuantizedFunctionGrid, name)) != null) {
      return new SpikingQuantizedFunctionGrid((PrototypedFunctionBuilder) getMapperBuilderFromName(params.get("innerMapper")));
    }
    //noinspection UnusedAssignment
    if ((params = params(directNumGrid, name)) != null) {
      return new DirectNumbersGrid();
    }
    //noinspection UnusedAssignment
    if ((params = params(functionNumGrid, name)) != null) {
      return new FunctionNumbersGrid();
    }
    throw new IllegalArgumentException(String.format("Unknown mapper name: %s", name));
  }

  @SuppressWarnings({"unchecked", "rawtypes"})
  private static Evolver<?, Robot<?>, Outcome> buildEvolver(String evolverName, String robotMapperName, Robot<?>
      target, Function<Outcome, Double>... outcomeMeasure) {
    if (outcomeMeasure.length == 0) {
      throw new IllegalArgumentException("At least one outcome measure needs to be specified");
    }
    PrototypedFunctionBuilder<?, ?> mapperBuilder = null;
    for (String piece : robotMapperName.split(MAPPER_PIPE_CHAR)) {
      if (mapperBuilder == null) {
        mapperBuilder = getMapperBuilderFromName(piece);
      } else {
        mapperBuilder = mapperBuilder.compose((PrototypedFunctionBuilder) getMapperBuilderFromName(piece));
      }
    }
    // TODO specify if measures need to be reversed or not
    PartialComparator<Outcome> comparator = PartialComparator.from(Double.class).comparing(outcomeMeasure[0]).reversed();
    for (int i = 1; i < outcomeMeasure.length; i++) {
      PartialComparator<Outcome> temporaryComparator = PartialComparator.from(Double.class).comparing(outcomeMeasure[i]);
      comparator = comparator.thenComparing(temporaryComparator);
    }
    return getEvolverBuilderFromName(evolverName).build(
        (PrototypedFunctionBuilder) mapperBuilder,
        target,
        comparator
    );
  }

  private static Function<Robot<?>, Outcome> buildTaskFromName(String transformationSequenceName, String
      terrainSequenceName, double episodeT, Random random, boolean outcomeCaching) {
    //for sequence, assume format '99:name>99:name'
    //transformations
    Function<Robot<?>, Robot<?>> transformation;
    if (transformationSequenceName.contains(SEQUENCE_SEPARATOR_CHAR)) {
      transformation = new SequentialFunction<>(getSequence(transformationSequenceName).entrySet().stream()
          .collect(Collectors.toMap(
                  Map.Entry::getKey,
                  e -> RobotUtils.buildRobotTransformation(e.getValue(), random)
              )
          ));
    } else {
      transformation = RobotUtils.buildRobotTransformation(transformationSequenceName, random);
    }
    //terrains
    Function<Robot<?>, Outcome> task;
    if (terrainSequenceName.contains(SEQUENCE_SEPARATOR_CHAR)) {
      task = new SequentialFunction<>(getSequence(terrainSequenceName).entrySet().stream()
          .collect(Collectors.toMap(
                  Map.Entry::getKey,
                  e -> buildLocomotionTask(e.getValue(), episodeT, random, outcomeCaching)
              )
          ));
    } else {
      task = buildLocomotionTask(terrainSequenceName, episodeT, random, outcomeCaching);
    }
    return task.compose(transformation);
  }

  public static Function<Robot<?>, Outcome> buildLocomotionTask(String terrainName, double episodeT, Random random, boolean cache) {
    if (!terrainName.contains("-rnd") && cache) {
      return Misc.cached(new Locomotion(
          episodeT,
          Locomotion.createTerrain(terrainName),
          PHYSICS_SETTINGS
      ), CACHE_SIZE);
    }
    return r -> new Locomotion(
        episodeT,
        Locomotion.createTerrain(terrainName.replace("-rnd", "-" + random.nextInt(10000))),
        PHYSICS_SETTINGS
    ).apply(r);
  }

  public static SortedMap<Long, String> getSequence(String sequenceName) {
    return new TreeMap<>(Arrays.stream(sequenceName.split(SEQUENCE_SEPARATOR_CHAR)).collect(Collectors.toMap(
        s -> s.contains(SEQUENCE_ITERATION_CHAR) ? Long.parseLong(s.split(SEQUENCE_ITERATION_CHAR)[0]) : 0,
        s -> s.contains(SEQUENCE_ITERATION_CHAR) ? s.split(SEQUENCE_ITERATION_CHAR)[1] : s
    )));
  }

}
