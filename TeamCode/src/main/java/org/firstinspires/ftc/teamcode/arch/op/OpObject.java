package org.firstinspires.ftc.teamcode.arch.op;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.util.Team;

import java.io.IOException;
import java.util.Enumeration;
import java.util.HashSet;

import dalvik.system.DexFile;

public abstract class OpObject extends OpMode {
    /*-------------------------------------------------------------
     * Static logic
     * -------------------------------------------------------------*/
    private final static HashSet<OpBehavior> s_globalBehaviors = new HashSet<>();
    private static OpObject s_current = null;
    public static void RegisterGlobalBehavior(OpBehavior behavior){
        s_globalBehaviors.add(behavior);
    }
    public final void UnregisterGlobalBehavior(OpBehavior behavior) {
        if(behavior.m_object == this) {
            s_globalBehaviors.remove(behavior);
            behavior.m_object = null;
        }
    }

    static {
        try {
            Context context = AppUtil.getDefContext();
            ClassLoader classLoader = context.getClassLoader();
            DexFile df = new DexFile(context.getPackageCodePath());
            try {
                for (Enumeration<String> iter = df.entries(); iter.hasMoreElements(); ) {
                    String className = iter.nextElement();
                    // 只掃描 teamcode 套件底下的類別，並跳過內部類別 (名稱含 $)
                    if (!className.startsWith("org.firstinspires.ftc.teamcode") || className.contains("$")) {
                        continue;
                    }

                    try {
                        // 1. 先載入類別結構但不執行 static 區塊 (initialize = false)
                        Class<?> clazz = Class.forName(className, false, classLoader);

                        // 2. 檢查是否為 OpBehavior 的具體子類別（排除抽象類別與接口）
                        if (OpBehavior.class.isAssignableFrom(clazz)
                                && ! java.lang.reflect.Modifier.isAbstract(clazz.getModifiers())) {
                            // 3. 正式初始化該類別 (initialize = true)，觸發其 static { RegisterGlobalBehavior(...); }
                            Class.forName(className, true, classLoader);
                        }
                    } catch (Throwable ignored) {
                        // 使用 Throwable 避免因為單一類別載入錯誤 (Error/Exception) 導致整個 App 崩潰
                    }
                }
            } finally {
                df.close();
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /*-------------------------------------------------------------
    * Instance logic
    * -------------------------------------------------------------*/
    private final HashSet<OpBehavior> m_behaviors = new HashSet<>();

    public abstract void Init();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();

    @Override
    public final void init() {
        s_current = this;
        Init();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Init();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            telemetry.addLine("init sub-system " + globalBehavior.getClass().getSimpleName());
            globalBehavior.m_object = this;
            globalBehavior.Init();
        }
    }

    @Override
    public void start() {
        Start();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Start();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Start();
        }
    }

    @Override
    public final void loop() {
        Loop();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Loop();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Loop();
        }
    }

    @Override
    public final void stop() {
        Stop();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Stop();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Stop();
            globalBehavior.m_object = null;
        }
        s_current = null;
    }

    public final void RegisterBehavior(OpBehavior behavior) {
        behavior.m_object = this;
        m_behaviors.add(behavior);
    }

    public final void UnregisterBehavior(OpBehavior behavior) {
        if(behavior.m_object == this) {
            m_behaviors.remove(behavior);
            behavior.m_object = null;
        }
    }

    public enum Mode{
        Competition, Test
    }
    private static Mode mode = Mode.Test;
    public static Mode getMode(){
        return mode;
    }
}
