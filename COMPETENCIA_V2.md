# Tequila 4.0 — Competencia v2

Guía de operación, configuración y tuning del robot de partido.
FRC 6702 Stingbots · Off-season 2026 · Limelight 4 fija en el cañón

---

## 1. Qué es esto

El código de Denver seguía funcionando, pero tenía tres cosas que ya habíamos
resuelto en el demo del StingyCamp y nunca migramos:

| Problema en Denver | Qué se hizo |
|---|---|
| El orbit era "muy violento": el PID de rumbo multiplicaba su salida por ~20 rad/s y saturaba | `HeadingController`: PD suave con la salida ya en rad/s, compuerta suave y derivativo filtrado |
| La torreta temblaba: se re-comandaba a 50 Hz con kI acumulando | `TurretAimTracker`: filtro + slew + deadband de comando; ganancias sin integral |
| El chasis respondía a jalones del stick con jalones de corriente | `SmoothDriveFilter` a velocidad de partido (100 %, precisión al 30 % con LT) |

Además:

- **Un solo estado de modos** (`CompetitionState`): STRIKER/BOMBER y HUB/FEEDER,
  en vez de tres enums estáticos que se pisaban desde cuatro botones.
- **Solución de tiro compartida** (`ShotSolution`): torreta, cañón fijo y chasis
  apuntan al mismo punto por construcción, con disparo en movimiento.
- **Limelight 4 fija** con su IMU interna para MegaTag2 y throttle térmico.
- **Agitación automática** de la caja al alimentar, **extensión con remate
  por corriente** (el intake sale completo aunque la calibración se corra) y
  **boost de rodillos por carga** (0.8 de base, 100 % cuando la caja llena lo
  pide).
- **Cámara de piloto** (la HBVCAM por USB) en Elastic.

Nada del demo se borró: `RobotConstants.isDemoMode = true` lo devuelve. Y los
bindings originales de Denver siguen en `RobotContainer.configureDenverBindings()`
(`useDenverBindings = true` los carga).

### Archivos nuevos

| Archivo | Qué es |
|---|---|
| `constants/CompetitionConstants.java` | Todo lo ajustable del modo partido |
| `constants/ShotConstants.java` | Física del tiro compartida por demo y competencia (tiempo de vuelo, disparo en movimiento, deadbands) |
| `util/CompetitionState.java` | STRIKER/BOMBER · HUB/FEEDER · objetivo activo |
| `util/HeadingController.java` | PD de rumbo suave |
| `util/TurretAimTracker.java` | Setpoints suaves de torreta |
| `util/MatchDashboard.java` | Publicación a Elastic (`Match/...`) |
| `commands/competition/CompDriveCommands.java` | Smooth drive + rumbo asistido en BOMBER |
| `commands/competition/CompTurretCommands.java` | Torreta por odometría + carga |
| `commands/competition/CompShooterCommands.java` | Cañón fijo en BOMBER |
| `commands/IntakeCommands.java` | Agitación y remate por corriente (ahora compartidos) |

### Archivos que cambiaron

- `RobotConstants` — `isDemoMode = false`, enum `CompetitionTarget`.
- `RobotContainer` — cableado de la LL4 fija, `configureCompetitionBindings()`.
- `Robot` — `MatchDashboard`, `CompetitionState.reset()` en teleop, cámara de piloto.
- `VisionConstants` — transformada de la LL4, modos de IMU, throttle.
- `Vision` / `VisionIOLimelight` — gestión del modo de IMU según el estado del robot.
- `TurretIOTalonFX` — ganancias v2 (sin kI) cuando `useV2TurretGains = true`.
- `Shooter`, `Drive`, `ShotSolution`, `FieldTracking` — leen de `ShotConstants`.
- `IntakeConstants` — constantes de agitación.
- `DemoConstants` — ahora **re-exporta** las constantes compartidas (no las edites ahí: son alias `final`).
- `elastic-layout.json` — pestaña **Competencia** nueva.

---

## 2. Flujo de operación

Es el del demo. Nada se mueve hasta que alguien lo pide.

```
1. SELECCIONAR MODO        (operador, Y)      STRIKER ↔ BOMBER
   SELECCIONAR OBJETIVO    (operador, Back)   HUB ↔ FEEDER
     No mueve nada. Sólo define qué hará el gatillo.

2. APUNTAR (X)  o  CARGAR (RT)
     X  → STRIKER: la torreta rastrea el objetivo por odometría.
          BOMBER:  la torreta va a cero y el CHASIS se alinea solo.
          Los flywheels NO giran.
     RT → lo mismo + hood y flywheel a la solución de tiro. Es "el gatillo".

3. ALIMENTAR               (operador, LB / RB — la Y-valve)
     LB = cañón fijo, RB = torreta, los dos = ambos.
     Mientras alimentas, la caja se agita sola.

Soltar todo → torreta a cero, flywheels apagados.
```

**Al dar enable el robot está en STRIKER → HUB.** `teleopInit` lo resetea a eso
siempre, así que el operador elige otra cosa a propósito, no por herencia del
partido anterior.

### Los cuatro estados

| Modo | Objetivo | Quién apunta | Quién dispara |
|---|---|---|---|
| STRIKER | HUB | Torreta | Torreta |
| STRIKER | FEEDER | Torreta (hacia la pared de la alianza) | Torreta |
| BOMBER | HUB | Chasis (trasera al HUB) | Los dos cañones |
| BOMBER | FEEDER | Chasis (trasera a la pared) | Los dos cañones |

BOMBER es el modo de volumen: dos cañones a la misma distancia. STRIKER es el
modo de "voy manejando y la torreta se encarga".

---

## 3. Mapa de controles

> Imprime esta sección y pégala en la caja del driveteam.

### Un control o dos

`CompetitionConstants.isSoloDriver`:

- `true` (**hoy**) — todo en el control del puerto 0: el mapeo del operador más
  el chasis en los sticks. Para probar solo en el taller. Cambian dos cosas:
  fijar el frente pasa de B a **POV ←** (B es retraer intake) y el modo
  precisión desaparece (LT es rodillos).
- `false` — piloto en el puerto 0, operador en el 1. **Es lo que va en partido.**

### Piloto — puerto 0

| Control | Acción |
|---|---|
| Stick izquierdo | Traslación. Field-relative con flip por alianza |
| Stick derecho | Giro. En BOMBER, mientras el operador apunta, el chasis se alinea solo; **mover el stick más de la mitad lo cancela** (válvula de escape) |
| LT | Modo precisión: 30 % |
| B | Reset del frente (0° azul / 180° rojo). Sirve deshabilitado. En modo solo: **POV ←** |

### Operador — puerto 1

| Control | Acción |
|---|---|
| Y | STRIKER ↔ BOMBER |
| Back | HUB ↔ FEEDER |
| X (hold) | APUNTAR |
| RT (hold) | CARGAR |
| LB / RB | Alimentar cañón fijo / torreta (+ agitación automática) |
| LT | Rodillos de intake: 0.8 de base + **boost al 100 %** cuando la corriente o la velocidad dicen que están empujando contra la caja llena |
| A (hold) | Extender intake con remate por corriente |
| B (hold) | Retraer intake |
| POV ↓ | Agitar la caja a mano |
| POV → | Desatascar indexer (reversa) |
| POV ↑ | Prueba de flywheels al 25 % (pit) |
| LS / RS | Jog del extensor **sin soft limits** (sólo recalibración) |
| Start | Libre |

---

## 4. Cómo apunta ahora

### La torreta apunta por pose, no por cámara

En Denver la Limelight estaba fija y la torreta apuntaba por odometría. Eso
**no cambió**; lo que cambió es cómo llega el ángulo al motor:

```
pose (encoders + Pigeon + MegaTag2)
  → ShotSolution.compute(pose, velocidad, objetivo)     ← distancia + delta angular
  → Turret.computeTurretAngleRad(pose, objetivo, delta) ← envuelve y recorta
  → TurretAimTracker: filtro → slew → deadband           ← lo que evita el temblor
  → rotateToAngle()
```

La visión **no** cierra ningún lazo sobre la torreta. Con MegaTag2 corrigiendo la
pose a 30+ fps, la odometría *es* la verdad y no hay "sesgo" que aprender como
en el demo (ahí la cámara iba en la torreta y la pose casi no se corregía). Si
la torreta apunta mal, el problema está en la pose o en la transformada de la
cámara, no en la torreta — ver sección 8.

### El chasis en BOMBER

Mismo principio: rumbo al objetivo + 180° (los cañones miran hacia atrás) + la
misma compensación de movimiento que usa la torreta. El rumbo objetivo tiene
slew (`headingTargetSlewRadPerSec`) para que una pose basura de un ciclo no
mande al robot a dar la vuelta, y muy cerca del objetivo se congela el último
rumbo bueno.

### Disparo en movimiento

`ShotSolution` corre el punto de mira en dirección contraria a la velocidad del
robot:

```
objetivoVirtual = objetivoReal − velocidad × tiempoDeVuelo
```

Resuelve ángulo y distancia a la vez. **Quedó encendido** desde la última
iteración del demo (`ShotConstants.shootWhileMovingEnabled = true`, ganancia
1.0). Antes de competir hay que verificarlo — sección 6.

---

## 5. Limelight 4

### Montaje

Va en el cañón fijo, mirando hacia **atrás** (por donde salen las pelotas). En
BOMBER, con la trasera al HUB, ve los tags del HUB de frente; en STRIKER ve lo
que haya detrás del robot. Como la torreta apunta por pose, la LL sólo tiene
que ver *algún* tag del campo de vez en cuando para mantener la odometría
corregida.

### Transformada

`VisionConstants.robotToLimelightFixed` se publica al arrancar
(`camerapose_robotspace_set`). **La pose de cámara en la web UI debe quedar en
ceros**; si dejas valores ahí se suman. Si prefieres configurarla en la web UI
como en Denver, pon `publishFixedCameraTransform = false`.

Los valores actuales son una estimación: `(-0.25, 0, 0.25)` m, pitch −20°
(mirando hacia arriba), yaw 180°. **Hay que medirlos** (sección 9).

### IMU

| Estado | `imumode_set` | Qué hace |
|---|---|---|
| Deshabilitado | 1 | Toma el yaw del Pigeon (`robot_orientation_set`) y siembra su IMU interna |
| Habilitado | 4 | Usa su IMU interna (menos latencia en giros) con el Pigeon como asistencia lenta |

Si la pose de visión se va girando sola estando quieto, cambia
`limelight4ImuModeEnabled` a 0 (sólo Pigeon, como la LL2).

### Throttle térmico

Deshabilitado salta 150 frames entre procesados (`limelight4ThrottleDisabled`);
habilitado procesa todos. La LL4 consume 12 W y en el pit se calienta.

### Pipeline

Un solo pipeline de AprilTags (0) a la resolución que dé ≥ 30 fps con los tags
del HUB detectados desde media cancha. En la LL4 eso suele ser 1280x960 con
decimate 2, o 960x720 con decimate 1. **Verifica `Match/Latencia ms` < 40** con
el robot en cancha.

---

## 6. Procedimiento de tuning

En orden. Cada paso depende del anterior.

### 6.1 Pose (primero, porque todo lo demás depende de esto)

1. Robot quieto en cancha viendo un tag del HUB. En AdvantageScope:
   `Vision/Camera0/RobotPosesAccepted` debe caer encima de `Odometry/Robot`.
2. Si la pose de visión está **corrida un offset fijo** → la traslación de la
   cámara está mal (`robotToLimelightFixed`, X/Y/Z).
3. Si la pose de visión **se mueve al girar el robot en su lugar** → el yaw
   está mal.
4. Si está bien quieto pero **se va al manejar** → latencia: revisa que el
   pipeline dé ≥ 30 fps.
5. Presiona B (reset de frente) y maneja: `Odometry/Robot` debe seguir al robot
   real sin brincos cuando aparece un tag.

### 6.2 Torreta

1. STRIKER → HUB, mantén X. `Match/Torreta` debe decir "HUB por odometría" y
   `Match/Error torreta deg` acercarse a 0 sin oscilar.
2. Si **tiembla en sitio** → sube `turretSetpointDeadbandRad`.
3. Si **se queda atrás al manejar** → sube `turretSetpointFilterAlpha` (más
   cerca de 1.0) o `turretMaxSetpointRateRadPerSec`.
4. Si **oscila alrededor del objetivo** → baja `turretGains.kP` o sube `kD`.
5. Si **apunta a un lado fijo** del HUB → la pose está corrida (6.1), no la
   torreta.

### 6.3 Rumbo de BOMBER

1. BOMBER → HUB, mantén X con el stick de giro suelto. El chasis debe girar
   hasta poner la trasera al HUB y quedarse ahí.
2. **Se pasa y regresa** → baja `headingKp` o sube `headingKd`.
3. **Se queda corto** (no llega a cero) → baja `headingToleranceRad` o sube
   `headingKp`.
4. **Vibra en el setpoint** → sube `headingToleranceRad` o `headingSoftZoneRad`.
5. **Muy lento** → sube `headingMaxOmegaRadPerSec` (tope 8).

### 6.4 Disparo en movimiento

Empieza con `shootWhileMovingEnabled = false` si vas a tirar estático el primer
día; enciéndelo cuando el tiro parado ya caiga.

1. Mide `kTimeOfFlightMap`: graba un tiro estático a 60 fps desde el costado,
   cuenta cuadros entre que la pelota sale y toca el HUB, divide entre 60.
   Repite a 2, 3, 4 y 5 m.
2. **Sin disparar**: STRIKER → HUB, mantén X, maneja **lateralmente a la
   derecha** viendo al HUB. `Match/Compensacion mov deg` debe ser **0 parado**
   y crecer **positivo** proporcional a `Match/Velocidad campo mps`.
   - Positivo → signo correcto.
   - Negativo → `shootWhileMovingAimSign = -1.0`.
   - No crece → revisa `Drive/FieldVelocityFiltered` en el log.
3. Ya con el número sano, dispara moviéndote. Si los tiros se corrigen pero
   quedan cortos, sube `shootWhileMovingGain` de 0.2 en 0.2.
4. Si el hood zumba → sube `hoodSetpointDeadbandDeg`; si la corrección tiembla
   → baja `fieldVelocityFilterAlpha`.

### 6.5 Intake

- **Boost de rodillos** (`rollersBoost*`): corre el intake al aire y anota
  `Intake/rollersCurrentAmps` (corriente libre); luego con la caja llena mira
  cuánto sube cuando le cuesta agarrar una pelota. `rollersBoostCurrentAmps` va
  a la mitad. En el log, `Intake/Boost/Phase` debe pasar a BOOST justo cuando
  ves que el rodillo se frena, y volver a BASE al soltar la pelota. Si se
  activa con la caja vacía → sube el umbral o `rollersBoostDebounceSeconds`; si
  nunca se activa → bájalo, o revisa `Intake/Boost/VelocityDroop` (la segunda
  señal). Si aun en BOOST no agarra, el límite es la corriente, no el duty: sube
  `rollersSupplyCurrentLimitAmps` de 30 a 40 (el breaker del intake lo aguanta).
- **No sale completo** → el remate por corriente lo resuelve; si no, sube
  `extendedRotation` **y** `extensorForwardLimitRad`.
- **El remate termina de inmediato** → `stallCurrentAmps` muy bajo (confunde el
  pico de arranque con el tope). Súbelo o sube `stallDebounceSeconds`.
- **Las pelotas hacen puente** → `agitateAmplitudeRad` o `agitateFrequencyHz`.

---

## 7. Checklist antes de un partido

**En el taller**

- [ ] `RobotConstants.isDemoMode = false`, `useDenverBindings = false`
- [ ] `CompetitionConstants.isSoloDriver = false` para partido (true sólo para probar solo)
- [ ] `./gradlew build` sin errores (el código se escribió sin compilar — ver sección 10)
- [ ] LL4: hostname `limelight-fixed`, pose de cámara en **ceros** en la web UI, pipeline 0 de AprilTags, LEDs apagados
- [ ] `VisionConstants.robotToLimelightFixed` medida (6.1)
- [ ] Prueba de inactividad: **habilita sin tocar nada — la torreta no debe moverse**
- [ ] Prueba STRIKER: mantén X, la torreta sigue el HUB al manejar sin temblar
- [ ] Prueba BOMBER: mantén X, el chasis se alinea; mover el stick fuerte lo cancela
- [ ] `Match/LISTO PARA TIRAR` se pone verde al cargar parado a 3 m
- [ ] `shootWhileMovingEnabled` verificado (6.4) o apagado
- [ ] Autos cargados en el chooser; cada uno probado al menos una vez
- [ ] Layout de Elastic cargado (`Ctrl + D` → "Load Layout From Robot"), pestaña **Competencia**
- [ ] Cámara de piloto visible en Elastic (widget "Piloto")
- [ ] Memoria USB montada para los logs

**En la cola de partido**

- [ ] Alianza correcta en la DS (el flip de campo y el HUB dependen de eso)
- [ ] Robot orientado hacia adelante y **B** presionado
- [ ] `Match/Limelight OK` en verde
- [ ] Auto seleccionado en el chooser

---

## 8. Diagnóstico

### 8.1 "La Limelight ve el tag pero el robot no apunta" — la cadena completa

Que la web UI de la Limelight dibuje el tag sólo prueba el primer eslabón. Hay
cinco, y la pestaña **Competencia** de Elastic los muestra en orden:

| Eslabón | Widget | Si falla |
|---|---|---|
| 1. La LL está en NetworkTables | `Limelight OK` | Hostname ≠ `limelight-fixed`, o el "Team Number" de la LL no es 6702 (no encuentra al Rio). Sin esto TODO lo demás está en gris |
| 2. Ve un tag | `Tag a la vista`, `Tag ID` | Pipeline equivocado (`Pipeline` debe ser 0, tipo AprilTag) |
| 3. Publica poses (botpose) | `Poses por ciclo` > 0 | La LL no tiene **mapa de campo 2026** cargado/seleccionado, o el pipeline no es de AprilTags. Sin mapa detecta el tag pero no calcula pose |
| 4. La pose se acepta | `Pose aceptada` verde; si no, `Ultimo rechazo` | `AMBIGUEDAD`: un solo tag visto de frente (MT1); normal, MT2 debería entrar. `Z ...`: la transformada de cámara está mal (pitch/altura). `FUERA DE CANCHA`: transformada o mapa incorrecto |
| 5. El rumbo es coherente | `Sembrando rumbo` parpadea en verde **deshabilitado** viendo el tag | Si nunca se pone verde, MT1 no pasa el filtro (ambigüedad): mira el tag en ángulo, no de frente |

Y dos que no son de visión pero se ven igual desde afuera:

- **`Alianza`**: la DS sin FMS arranca en Rojo. Con un tag azul y alianza roja
  el robot apunta al HUB del otro lado del campo. Ponla en Azul en la DS.
- **El tag tiene que ser del HUB** (IDs 2-5, 8-11, 18-21, 24-27) para que
  apuntar "al HUB" coincida con apuntar al tag. Con cualquier otro ID el robot
  calcula dónde estaría el HUB respecto a ese tag y apunta ahí, que en el
  taller es una pared cualquiera. Es correcto, pero parece roto.

**Por qué el rumbo importa tanto:** MegaTag2 nunca corrige rotación; confía en
el yaw del Pigeon y resuelve sólo posición. En cancha eso funciona porque B fija
el frente mirando al lado contrario. En el taller, con el robot en cualquier
orientación respecto a un tag en la pared, el yaw no tiene relación con el marco
del tag, MegaTag2 calcula una posición "coherente" con ese yaw equivocado y la
torreta apunta a un HUB imaginario. Por eso ahora, mientras el robot está
**deshabilitado** y ve un tag, MegaTag1 reescribe posición y rumbo
(`seedHeadingFromVisionWhileDisabled`). Procedimiento en el taller: pon el
robot viendo el tag **en ángulo** (no de frente), espera a que `Sembrando
rumbo` parpadee, y habilita. `Odometry/Robot` en AdvantageScope debe quedar
frente al tag en el mapa.

### 8.2 Tabla de síntomas

| Síntoma | Dónde mirar |
|---|---|
| La torreta no hace nada | Correcto si no mantienes X o RT |
| La torreta apunta a un lado fijo del HUB | Pose corrida. `Vision/Camera0/RobotPosesAccepted` vs `Odometry/Robot`. Sección 6.1 |
| La torreta se rezaga al manejar | `turretSetpointFilterAlpha` ↑ o `turretMaxSetpointRateRadPerSec` ↑ |
| La torreta tiembla | `turretSetpointDeadbandRad` ↑. Si oscila con amplitud, kP ↓ |
| "ODOMETRIA VIEJA" en el estado | Más de 8 s sin un tag aceptado. La torreta sigue apuntando; revisa `Match/Tag a la vista` y la exposición del pipeline |
| BOMBER no gira el chasis | ¿Estás manteniendo X o RT? ¿El stick de giro está más de la mitad? `Match/Fuente rumbo` en MANUAL (piloto) = lo cancelaste tú |
| BOMBER gira al lado contrario / se queda a 180° | El objetivo es rumbo + 180° porque los cañones miran atrás. Si el robot cambió de geometría, revisa `Rotation2d.k180deg` en `TargetHeading` |
| El hood no se mueve al cargar | `Match/Distancia m`: si marca < 1 m lejos del HUB, la pose no está inicializada. Presiona B y deja que vea un tag |
| Los dos hoods marcan distinto en BOMBER | No debería: comparten `ShotSolution`. Revisa que ningún comando viejo (`ShooterCommands`/`TurretCommands`) esté enlazado |
| La compensación se va al revés | Sección 6.4, paso 2. Mira el signo ANTES de tocar `shootWhileMovingAimSign` |
| El intake no sale completo | `extendedRotation` + `extensorForwardLimitRad`, o deja que el remate lo resuelva |
| El rodillo no agarra con la caja llena | `Intake/Boost/Phase`: si no llega a BOOST, baja `rollersBoostCurrentAmps`; si llega y aun así no agarra, sube `rollersSupplyCurrentLimitAmps` |
| El rodillo se pone al 100 % con la caja vacía | `rollersBoostCurrentAmps` ↑ o `rollersBoostDebounceSeconds` ↑ |
| La pose de visión gira sola quieto | `limelight4ImuModeEnabled = 0` |
| Latencia > 60 ms | Baja resolución o sube decimate en el pipeline |
| Sin video de piloto | `CompetitionConstants.useDriverCamera`, y que la HBVCAM esté en el puerto USB del Rio (dev 0) |
| El robot se siente "flotado" al arrancar | `translationMaxAccel` ↑ |
| Las llantas patinan al arrancar | `translationMaxAccel` ↓ |

---

## 9. Calibrar la geometría de la Limelight

Cinta métrica y nivel digital.

1. **Origen**: el centro del robot es el centro del cuadrado que forman las
   cuatro ruedas (no el centro de los bumpers).
2. **X**: distancia del origen al lente, positivo hacia adelante. La cámara
   está atrás → negativo.
3. **Y**: positivo hacia la izquierda del robot (visto desde atrás hacia
   adelante).
4. **Z**: altura del lente sobre el piso.
5. **Pitch**: inclinación hacia arriba del lente, **negativa** en la convención
   WPILib (−20° = mira 20° hacia arriba).
6. **Yaw**: π (180°) porque mira hacia atrás. Si está ligeramente girada,
   suma/resta ese ángulo.
7. Prueba decisiva: sección 6.1.

---

## 10. Estado del código

- Todos los archivos parsean sin errores de sintaxis (tree-sitter).
- Cero referencias sin resolver a clases, constantes y métodos del proyecto
  (verificación cruzada automática).
- `elastic-layout.json` valida como JSON (3 pestañas).
- **No se pudo compilar con Gradle**: el entorno no tiene acceso al Maven de
  WPILib. **Corre `./gradlew build` antes de desplegar.** Lo más probable que
  aparezca son imports sin usar (warnings) o alguna firma de WPILib que cambió
  de nombre; los errores de lógica ya se revisaron.

### Pendientes de verificar en el robot, en orden

1. **`robotToLimelightFixed`** — medir (sección 9). Bloquea todo lo demás.
2. **Pipeline de la LL4** — resolución/decimate para ≥ 30 fps, latencia < 40 ms.
3. **Modo de IMU 4** — que la pose no gire sola; si sí, modo 0.
4. **Torreta**: `turretGains` (kP 45 sin kI) y el deadband. Si prefieres
   arrancar con lo de Denver, `useV2TurretGains = false`.
5. **Rumbo de BOMBER**: `headingKp` 3.0 / `headingKd` 0.35.
6. **Disparo en movimiento**: signo y ganancia (6.4). Apágalo si no hay tiempo.
7. **Mapas de tiro** (`ShooterConstants.kShooterHoodMap` / `kShooterFlywheelMap`)
   — son los de Denver, verificar que siguen cayendo con el robot actual.
8. **Cámara de piloto**: ancho de banda en la DS < 3 Mbps con el stream abierto.

### Qué NO cambió

- Los autos de PathPlanner y sus `NamedCommands` (`AimTurret`, `IndexTurret`,
  `ExtendIntake`, ...).
- `IndexerCommands` (la Y-valve).
- Los mapas de tiro de competencia.
- El código del demo: sigue detrás de `isDemoMode = true`.
