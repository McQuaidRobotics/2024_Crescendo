package com.igknighters.constants;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import com.igknighters.constants.RobotConfig.RobotID;
import com.igknighters.util.logging.BootupLogger;

import edu.wpi.first.wpilibj.DriverStation;

public class ConstantHelper {

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface BoolConst {

        public boolean crash(); // default false;

        public boolean burn(); // default false;

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface IntConst {

        public int crash(); // default 0;

        public int burn(); // default 0;

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface LongConst {

        public long crash(); // default 0;

        public long burn(); // default 0;

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface FloatConst {

        public float crash(); // default 0f;

        public float burn(); // default 0f;

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface DoubleConst {

        public double crash(); // default 0d;

        public double burn(); // default 0d;

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface StrConst {

        public String crash(); // default "";

        public String burn(); // default "";

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface BoolArrConst {

        public boolean[] crash(); // default {};

        public boolean[] burn(); // default {};

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface IntArrConst {

        public int[] crash(); // default {};

        public int[] burn(); // default {};

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface LongArrConst {

        public long[] crash(); // default {};

        public long[] burn(); // default {};

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface FloatArrConst {

        public float[] crash(); // default {};

        public float[] burn(); // default {};

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface DoubleArrConst {

        public double[] crash(); // default {};

        public double[] burn(); // default {};

    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface StrArrConst {

        public String[] crash(); // default {};

        public String[] burn(); // default {};

    }

    public static void handleConstField(Field field, Class<?> obj, RobotID id) {
        if (field.getAnnotations().length == 0) {
            return;
        }
        RobotConstID constID = id.constID;

        // handle robot dependent constants

        if (field.isAnnotationPresent(BoolConst.class)) {
            try {
                BoolConst annotation = field.getAnnotation(BoolConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(IntConst.class)) {
            try {
                IntConst annotation = field.getAnnotation(IntConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(LongConst.class)) {
            try {
                LongConst annotation = field.getAnnotation(LongConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(FloatConst.class)) {
            try {
                FloatConst annotation = field.getAnnotation(FloatConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(DoubleConst.class)) {
            try {
                DoubleConst annotation = field.getAnnotation(DoubleConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(StrConst.class)) {
            try {
                StrConst annotation = field.getAnnotation(StrConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(BoolArrConst.class)) {
            try {
                BoolArrConst annotation = field.getAnnotation(BoolArrConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(IntArrConst.class)) {
            try {
                IntArrConst annotation = field.getAnnotation(IntArrConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(LongArrConst.class)) {
            try {
                LongArrConst annotation = field.getAnnotation(LongArrConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(FloatArrConst.class)) {
            try {
                FloatArrConst annotation = field.getAnnotation(FloatArrConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(DoubleArrConst.class)) {
            try {
                DoubleArrConst annotation = field.getAnnotation(DoubleArrConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        if (field.isAnnotationPresent(StrArrConst.class)) {
            try {
                StrArrConst annotation = field.getAnnotation(StrArrConst.class);

                if (constID == RobotConstID.CRASH) {
                    field.set(obj, annotation.crash());
                }

                if (constID == RobotConstID.BURN) {
                    field.set(obj, annotation.burn());
                }

            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        // makes sure its an NT supported type
        var type = field.getType();
        if (type.isArray()) {
            type = type.getComponentType();
        }
        if (!(type.isPrimitive() || type == String.class)) {
            return;
        }
    }

    public static void handleConstSubclass(Class<?> cls, RobotID id) {
        for (Class<?> clazz : cls.getDeclaredClasses()) {
            handleConstSubclass(clazz, id);
        }
        if (Modifier.isAbstract(cls.getModifiers())) {
            return;
        }
        for (Field field : cls.getDeclaredFields()) {
            if (!Modifier.isStatic(field.getModifiers()) && !cls.isEnum()) {
                DriverStation.reportError("Non-static field " + cls.getSimpleName() + "." + field.getName()
                        + " in constants", false);
                continue;
            }
            handleConstField(field, cls, id);
        }
    }

    public static void applyRoboConst(Class<com.igknighters.constants.ConstValues> consts, RobotID id) {
        for (Class<?> clazz : consts.getDeclaredClasses()) {
            handleConstSubclass(clazz, id);
        }
        for (Field field : consts.getDeclaredFields()) {
            handleConstField(field, consts, id);
        }
        BootupLogger.bootupLog("Applied Robot Constants");
    }

    public enum RobotConstID {
        CRASH,
        BURN,;
    }
}
