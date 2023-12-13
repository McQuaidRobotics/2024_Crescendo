package com.igknighters.constants;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

import com.igknighters.constants.RobotSetup.RobotConstID;
import com.igknighters.util.BootupLogger;
import edu.wpi.first.wpilibj.DriverStation;

public class ConstantHelper {

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface IntConst {
        int yin();

        int yang();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface IntArrayConst {
        int[] yin();

        int[] yang();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface DoubleConst {
        double yin();

        double yang();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface DoubleArrayConst {
        double[] yin();

        double[] yang();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface StringConst {
        String yin();

        String yang();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface StringArrayConst {
        String[] yin();

        String[] yang();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface BooleanConst {
        boolean yin();

        boolean yang();
    }

    @Retention(RetentionPolicy.RUNTIME)
    @Target({ ElementType.FIELD })
    public @interface BooleanArrayConst {
        boolean[] yin();

        boolean[] yang();
    }

    // /**
    //  * Still puts the value on network tables but changing it doesn't change the
    //  * const value
    //  */
    // @Retention(RetentionPolicy.RUNTIME)
    // @Target({ ElementType.TYPE, ElementType.FIELD })
    // public @interface TunableIgnore {
    // }

    // /** Doesn't put the value on network tables at all */
    // @Retention(RetentionPolicy.RUNTIME)
    // @Target({ ElementType.TYPE, ElementType.FIELD })
    // public @interface NTIgnore {
    // }

    // @Retention(RetentionPolicy.RUNTIME)
    // @Target({ ElementType.TYPE, ElementType.FIELD })
    // public @interface NTPreferenceConst {
    //     String value() default "";
    // }

    public static void handleConstField(Field field, Class<?> obj) {
        if (field.getAnnotations().length == 0) {
            return;
        }
        RobotConstID constID = RobotSetup.getRobotID().constID;

        //handle robot dependent constants
        if (field.isAnnotationPresent(IntConst.class)) {
            try {
                IntConst annotation = field.getAnnotation(IntConst.class);
                if (constID == RobotConstID.YIN) {
                    field.set(obj, annotation.yin());
                } else if (constID == RobotConstID.YANG) {
                    field.set(obj, annotation.yang());
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        } else if (field.isAnnotationPresent(DoubleConst.class)) {
            try {
                DoubleConst annotation = field.getAnnotation(DoubleConst.class);
                if (constID == RobotConstID.YIN) {
                    field.set(obj, annotation.yin());
                } else if (constID == RobotConstID.YANG) {
                    field.set(obj, annotation.yang());
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        } else if (field.isAnnotationPresent(StringConst.class)) {
            try {
                StringConst annotation = field.getAnnotation(StringConst.class);
                if (constID == RobotConstID.YIN) {
                    field.set(obj, annotation.yin());
                } else if (constID == RobotConstID.YANG) {
                    field.set(obj, annotation.yang());
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        } else if (field.isAnnotationPresent(BooleanConst.class)) {
            try {
                BooleanConst annotation = field.getAnnotation(BooleanConst.class);
                if (constID == RobotConstID.YIN) {
                    field.set(obj, annotation.yin());
                } else if (constID == RobotConstID.YANG) {
                    field.set(obj, annotation.yang());
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        } else if (field.isAnnotationPresent(IntArrayConst.class)){
            try {
                IntArrayConst annotation = field.getAnnotation(IntArrayConst.class);
                if (constID == RobotConstID.YIN) {
                    field.set(obj, annotation.yin());
                } else if (constID == RobotConstID.YANG) {
                    field.set(obj, annotation.yang());
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        } else if (field.isAnnotationPresent(DoubleArrayConst.class)){
            try {
                DoubleArrayConst annotation = field.getAnnotation(DoubleArrayConst.class);
                if (constID == RobotConstID.YIN) {
                    field.set(obj, annotation.yin());
                } else if (constID == RobotConstID.YANG) {
                    field.set(obj, annotation.yang());
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        } else if (field.isAnnotationPresent(StringArrayConst.class)){
            try {
                StringArrayConst annotation = field.getAnnotation(StringArrayConst.class);
                if (constID == RobotConstID.YIN) {
                    field.set(obj, annotation.yin());
                } else if (constID == RobotConstID.YANG) {
                    field.set(obj, annotation.yang());
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        } else if (field.isAnnotationPresent(BooleanArrayConst.class)){
            try {
                BooleanArrayConst annotation = field.getAnnotation(BooleanArrayConst.class);
                if (constID == RobotConstID.YIN) {
                    field.set(obj, annotation.yin());
                } else if (constID == RobotConstID.YANG) {
                    field.set(obj, annotation.yang());
                }
            } catch (IllegalAccessException e) {
                e.printStackTrace();
            }
        }

        //makes sure its an NT supported type
        var type = field.getType();
        if (type.isArray()) {
            type = type.getComponentType();
        }
        if (!(type.isPrimitive() || type == String.class)) {
            return;
        }
    }

    public static void handleConstSubclass(Class<?> cls) {
        for (Class<?> clazz : cls.getDeclaredClasses()) {
            handleConstSubclass(clazz);
        }
        if (Modifier.isAbstract(cls.getModifiers())) {
            return;
        }
        for (Field field : cls.getDeclaredFields()) {
            if (!Modifier.isStatic(field.getModifiers())) {
                DriverStation.reportError("Non-static field " + cls.getSimpleName() + "." + field.getName()
                        + " in constants", false);
                continue;
            }
                handleConstField(field, cls);
            }
        }
    

    public static void applyRoboConst(Class<ConstValues> consts) {
        for (Class<?> clazz : consts.getDeclaredClasses()) {
            handleConstSubclass(clazz);
        }
        for (Field field : consts.getDeclaredFields()) {
            handleConstField(field, consts);
        }
        BootupLogger.BootupLog("Finished applying constants");
    }
}
