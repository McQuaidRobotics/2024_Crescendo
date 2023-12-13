package frc.robot.util;

import java.util.List;
import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Robot;

public class ErrorHelper {

    public interface Error {

        /** Not meant to be called outside this class */
        public String msg();

        /**
         * How the error should be printed in logs,
         * default is "<class_name>: <err msg>",
         */
        default public String display() {
            return this.getClass().toGenericString() + ": " + msg();
        }

        public static Error createCustom(String message) {
            return new Error() {
                @Override
                public String msg() {
                    return message;
                }
            };
        }

        public static Error createCustom(Exception e) {
            return new Error() {
                @Override
                public String msg() {
                    return e.getMessage();
                }
            };
        }

        default public void log() {
            if (Robot.isSimulation()) {
                System.out.println(this.display());
            } else {
                DriverStation.reportError(this.display(), false);
            }
        }

        default public Result<?, Error> toResultWeak() {
            return Result.err(this);
        }

        default public <T> Result<T, Error> toResultStrong() {
            return Result.err(this);
        }
    }

    public static interface ErrorEnum {
        /**
         * A way to implement display for each error variant in the enum
         * 
         * @param errMsg the return of {@link Error#msg()}
         * @return your custom display text
         */
        default public String displayVariant(String errMsg) {
            return null;
        }
    }

    public interface GroupError<T extends Enum<T> & ErrorEnum> extends Error {
        /**
         * @return the associated enum class
         */
        @SuppressWarnings("unchecked")
        default public Class<T> getEnum() {
            return (Class<T>) getVariant().getClass();
        }

        /**
         * @return if the error belongs to the provided enum
         */
        default public Boolean isOfEnum(Class<?> clazz) {
            return clazz.getCanonicalName() == getEnum().getCanonicalName();
        }

        /**
         * @return the associated enum variant
         */
        public T getVariant();

        /**
         * How the error should be printed in logs,
         * default is "<enum_name>(<variant_name>: <err msg>)",
         * this can be overriden in the enum or in this error,
         * note: the errors override has priority
         */
        @Override
        default String display() {
            if (getVariant().displayVariant("") == null) {
                return getEnum().getTypeName() + "(" + getVariant().name() + ": " + msg() + ")";
            } else {
                return getVariant().displayVariant(this.msg());
            }
        }
    }

    /**
     * An object containing always 1 of 2 potential values. <p>
     * The generic types define what the result can contain,
     * the first type is the "ok" value type(no constraints), the second is the "err" value type(has to implement {@link Error}).
     */
    public static class Result<T, E extends Error> {
        public final Optional<T> okValue;
        public final Optional<E> errValue;

        public Result(Optional<T> ok, Optional<E> err) {
            this.okValue = ok;
            this.errValue = err;
        }

        public static <T, ER extends Error> Result<T, ER> err(ER error) {
            return new Result<T, ER>(Optional.empty(), Optional.of(error));
        }

        public static <T, ER extends Error> Result<T, ER> ok(T okay) {
            return new Result<T, ER>(Optional.of(okay), Optional.empty());
        }

        public boolean isErr() {
            return errValue.isPresent();
        }

        public boolean isOk() {
            return okValue.isPresent();
        }

        public T unwrap() {
            return okValue.get();
        }

        public E unwrapErr() {
            return errValue.get();
        }

        public T unwrapOr(T defaultVal) {
            return okValue.orElse(defaultVal);
        }

        public void logIfErr() {
            if (isErr()) {
                this.unwrapErr().log();
            }
        }
    }

    /**
     * A completely empty class just mean to fill a generic
     * for result if the function would normally return void
     */
    public static class Ok {
        public Ok() {}
    }

    /**
     * An error type that should never occur, throws an exception if it does occur
     */
    public static class Infallible implements Error {
        public Infallible() {
            throw new RuntimeException("Infallible error was called");
        }

        @Override
        public String msg() { return ""; }
    }

    public static class ErrorList implements Error {
        private final List<Error> errors;

        public ErrorList(Error... errors) {
            this.errors = List.of(errors);
        }

        public void append(Error error) {
            this.errors.add(error);
        }

        public void extend(ErrorList errorList) {
            this.errors.addAll(errorList.errors);
        }

        public void extend(Error... errors) {
            this.errors.addAll(List.of(errors));
        }

        public void appendResult(Result<?, Error> result) {
            if (result.isErr()) {
                this.errors.add(result.unwrapErr());
            }
        }

        public void clear() {
            this.errors.clear();
        }

        @Override
        @Deprecated
        /** Dont call this on ErrorList */
        public String msg() {
            return "msg cannot be called on this class";
        }

        @Override
        public String display() {
            StringBuilder sb = new StringBuilder();
            for (Error error : errors) {
                sb.append(error.display());
                sb.append("\n");
            }
            return sb.toString();
        }
    }
}
