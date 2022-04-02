package it.units.erallab.evolution.builder;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Collectors;

/**
 * @author eric
 */
public interface PrototypedFunctionBuilder<A, B> {
  Function<A, B> buildFor(B b);

  A exampleFor(B b);

  default <C> PrototypedFunctionBuilder<C, B> compose(PrototypedFunctionBuilder<C, A> other) {
    PrototypedFunctionBuilder<A, B> thisB = this;
    return new PrototypedFunctionBuilder<>() {
      @Override
      public Function<C, B> buildFor(B b) {
        return thisB.buildFor(b).compose(other.buildFor(thisB.exampleFor(b)));
      }

      @Override
      public C exampleFor(B b) {
        return other.exampleFor(thisB.exampleFor(b));
      }
    };
  }

  static <A1, B1> PrototypedFunctionBuilder<List<A1>, List<B1>> of(List<PrototypedFunctionBuilder<A1, B1>> builders) {
    return new PrototypedFunctionBuilder<>() {
      @Override
      public Function<List<A1>, List<B1>> buildFor(List<B1> b1s) {
        if (b1s.size() != builders.size()) {
          throw new IllegalArgumentException(String.format(
              "Wrong number of arguments: %d expected, %d found",
              builders.size(),
              b1s.size()
          ));
        }
        return a1s -> {
          if (a1s.size() != builders.size()) {
            throw new IllegalArgumentException(String.format(
                "Wrong number of arguments: %d expected, %d found",
                builders.size(),
                a1s.size()
            ));
          }
          List<B1> newB1s = new ArrayList<>(builders.size());
          for (int i = 0; i < builders.size(); i++) {
            newB1s.add(builders.get(i).buildFor(b1s.get(i)).apply(a1s.get(i)));
          }
          return newB1s;
        };
      }

      @Override
      public List<A1> exampleFor(List<B1> b1s) {
        if (b1s.size() != builders.size()) {
          throw new IllegalArgumentException(String.format(
              "Wrong number of arguments: %d expected, %d found",
              builders.size(),
              b1s.size()
          ));
        }
        List<A1> a1s = new ArrayList<>(builders.size());
        for (int i = 0; i < builders.size(); i++) {
          a1s.add(builders.get(i).exampleFor(b1s.get(i)));
        }
        return a1s;
      }
    };
  }

  static <T> PrototypedFunctionBuilder<List<T>, List<List<T>>> merger() {
    return new PrototypedFunctionBuilder<>() {
      @Override
      public Function<List<T>, List<List<T>>> buildFor(List<List<T>> lists) {
        return ts -> {
          List<List<T>> newLists = new ArrayList<>(lists.size());
          int sum = lists.stream().mapToInt(List::size).sum();
          if (ts.size() != sum) {
            throw new IllegalArgumentException(String.format(
                "Not enough values: %d instead of %d",
                ts.size(),
                sum
            ));
          }
          int c = 0;
          for (List<T> list : lists) {
            newLists.add(ts.subList(c, c + list.size()));
            c = c + list.size();
          }
          return newLists;
        };
      }

      @Override
      public List<T> exampleFor(List<List<T>> lists) {
        return lists.stream().flatMap(List::stream).collect(Collectors.toList());
      }
    };
  }
}
