# Tequila 4.0 — Demo Mode

Guía de operación, configuración y tuning.
FRC 6702 Stingbots · Temporada 2026 · Limelight 2/2+ en torreta

---

## 1. Interruptor maestro

```java
// src/main/java/frc/robot/constants/RobotConstants.java
public static final boolean isDemoMode = true;   // demo / competencia
public static final boolean isSoloDemo = true;   // un solo control / dos
```

Nada del código de competencia fue borrado ni modificado en su comportamiento.
El flag decide qué default commands y qué bindings se cargan.

### Archivos del demo

| Archivo | Qué es |
|---|---|
| `constants/DemoConstants.java` | Todo lo ajustable, en un solo lugar |
| `util/SmoothRateLimiter.java` | Limitador escalar de segundo orden (curva en S) |
| `util/SmoothDriveFilter.java` | Filtro vectorial de traslación |
| `util/DemoState.java` | Estado de modos + transiciones |
| `util/DemoDashboard.java` | Publicación a Elastic |
| `commands/demo/DemoDriveCommands.java` | Smooth drive, hub orbit, follow-me |
| `commands/demo/DemoTurretCommands.java` | Activación, rastreo y carga |
| `commands/demo/DemoShooterCommands.java` | Cañón fijo |
| `commands/demo/DemoIntakeCommands.java` | Agitación de la caja |
| `src/main/deploy/elastic-layout.json` | Layout de Elastic listo para importar |

---

## 2. Flujo de operación

**Éste es el cambio grande respecto a la versión anterior.** Antes la torreta se
ponía a barrer sola en cuanto dabas enable. Ahora no se mueve hasta que alguien
se lo pide, igual que en competencia.

```
1. SELECCIONAR MODO        (operador, botón Y)
     No mueve nada. Sólo le dice al robot qué va a hacer el gatillo.

2. APUNTAR  (X)  o  CARGAR  (RT)
     X  → la torreta rastrea. Los flywheels NO giran.
     RT → rastrea Y acelera a la solución de tiro. Es "el gatillo".
          Mientras lo mantengas, el cañón queda listo y apuntando.

3. ALIMENTAR               (operador, LB / RB — la Y-valve)
     La pelota entra a un cañón que ya está acelerado y apuntando.

Soltar todo → torreta a cero, flywheels apagados, nada buscando.
```

El barrido de búsqueda **sólo** ocurre mientras mantienes APUNTAR o CARGAR y no
hay ningún tag a la vista. Con el robot habilitado y sin botones, la torreta se
queda quieta.

Lo mismo aplica al chasis: en BOMBER el auto-alineado al HUB **sólo** se activa
mientras el operador apunta o carga. Antes, cambiar a BOMBER le quitaba el giro
al piloto de golpe sin que nadie lo hubiera pedido.

---

## 3. Mapa de controles

> Imprime esta sección y pégala en la caja del driveteam.

### Modo solo (`isSoloDemo = true`) — un solo control

Todo va al control del **puerto 0**. El puerto 1 queda sin usar.

| Control | Qué hace |
|---|---|
| **Sticks** | Chasis: traslación e giro |
| **Y** | Alterna STRIKER ↔ BOMBER |
| **Back** | Smart feature del modo activo |
| **X** (mantener) | APUNTAR — torreta rastrea, sin flywheels |
| **RT** (mantener) | CARGAR (el gatillo) — rastrea + acelera |
| **LB / RB** (mantener) | Alimentar cañón fijo / torreta |
| **LT** (mantener) | Rodillos de intake |
| **A / B** (mantener) | Extender (con remate por corriente) / retraer intake |
| **Start** | Alterna orientación FIELD ↔ DRIVER |
| **POV ←** | Fijar el frente |
| **POV ↑** | Prueba de flywheels al 25% |
| **POV ↓** | Agitación manual |
| **POV →** | Desatascar indexer |
| **LS / RS** (click) | Jog del extensor sin soft limits |

Cambios respecto al mapeo de dos controles:

- El toggle de orientación (era **X** del piloto) se fue a **Start**, y con eso
  desaparece el ciclado manual de pipelines de la Limelight.
- Fijar el frente (era **B** del piloto) se fue a **POV izquierda**.
- El modo precisión (era **LT** del piloto) **desaparece**: LT es los rodillos.
- En BOMBER la smart feature es el **volcado suave**, no el perrito.

---

### Modo dos controles (`isSoloDemo = false`)

#### Piloto — puerto 0

| Control | Qué hace |
|---|---|
| **Stick izquierdo** | Traslación. Suavizada. Sin flip de alianza. |
| **Stick derecho (X)** | Giro. Salvo que el chasis esté asistido. |
| **LT** | Modo precisión — baja al 25%. |
| **X** | Alterna orientación **FIELD** ↔ **DRIVER** |
| **B** | Fija el frente (su efecto depende del modo, ver abajo) |

Cinco cosas. Es todo lo que le vas a explicar a un alumno nuevo.

**Los dos modos de orientación:**

| Modo | Cómo funciona | Qué hace B |
|---|---|---|
| **FIELD** (default) | Igual que competencia: field-relative con la rotación de la odometría y flip por alianza. El "adelante" del stick es el adelante del campo. | Reset de giroscopio de competencia (reescribe la pose) |
| **DRIVER** | El piloto declara su propio frente y ése manda. | Guarda un **offset**, sin tocar la odometría |

> **Por qué existe el modo DRIVER.** Antes B hacía `drive.setPose()`, que
> reescribe la rotación de la odometría. Un segundo después llegaba una
> corrección de visión, devolvía la pose a la verdad del campo, y se llevaba el
> frente que acababas de fijar — de ahí el "presiono B y cuando ve un AprilTag
> se me mueve el frente completo".
>
> La solución no es pelearse con el pose estimator: en modo DRIVER, B guarda un
> **offset**. La visión sigue corrigiendo la odometría (que es lo que el
> apuntado necesita) y tu frente se calcula como `rotación − offset`. Como los
> dos términos se mueven juntos, la diferencia no cambia.

### Operador — puerto 1

| Control | Qué hace |
|---|---|
| **Y** | Alterna **STRIKER** ↔ **BOMBER** |
| **Back** | Smart feature del modo activo |
| **X** (mantener) | **APUNTAR** — la torreta rastrea, sin flywheels |
| **RT** (mantener) | **CARGAR** (el gatillo) — rastrea + acelera |
| **LB** (mantener) | Alimentar cañón fijo (Y-valve) |
| **RB** (mantener) | Alimentar torreta (Y-valve) |
| **LB + RB** | Repartir a ambos |
| **LT** (mantener) | Rodillos de intake |
| **A** (mantener) | Extender intake — con remate por corriente hasta el tope |
| **B** (mantener) | Retraer intake |
| **Start** | Rota el pipeline de la Limelight a mano |
| **POV ↑** | Prueba de flywheels al 25% (pit) |
| **POV ↓** | Agitación manual de la caja |
| **POV →** | Desatascar indexer (reversa) |
| **LS / RS** (click) | Jog del extensor sin soft limits (recalibración) |

**Botón `Back` según el modo:**

- **STRIKER** → alterna entre **HUB** (default, potencia de competencia) y
  **caza libre** de cualquier AprilTag (tiro suave). Si hay un tag a la vista al
  activar la caza libre, se engancha a ése.
- **BOMBER** → alterna entre orbitar el HUB y el **follow-me** ("perrito").

> El toggle ahora **siempre** alterna, sin condiciones. Antes sólo entraba a
> fijar tag si había uno visible en ese instante, así que presionarlo no hacía
> nada visible y parecía descompuesto.

**¿Por qué X apunta sin cargar?** Porque caminando por la prepa con alumnos
alrededor, lo que quieres mostrar es la torreta siguiendo un tag — no dos
volantes de inercia girando a 2000 RPM cerca de la gente. X es el modo seguro de
exhibición; RT es cuando de verdad vas a tirar.

---

## 4. Los dos modos

### STRIKER

**Torreta:** rastrea mientras mantengas X o RT. **Chasis:** 100% manual.

STRIKER tiene dos sub-modos con **dos lógicas de apuntado distintas**, a
propósito:

| | Cómo apunta | Distancia | Potencia |
|---|---|---|---|
| **HUB** (default) | Visión → gracia → odometría → barrido (ver abajo) | Odometría | Tus mapas de competencia, con hood ajustable |
| **Caza libre** (Back) | La misma máquina, sin el nivel de odometría | Visión | Mapa de tiro suave |

### La máquina de estados de la torreta

Es **una sola** para los dos sub-modos, y los cuatro niveles se evalúan en este
orden cada ciclo:

| # | Estado | Se activa cuando | Qué hace |
|---|---|---|---|
| 1 | **VISIÓN** | Ve un tag aceptado | Cierra el lazo sobre `tx`, con histéresis y compensación de latencia |
| 2 | **GRACIA** | Perdió el tag hace menos de `turretTargetGraceSeconds` | Mantiene el último apuntado |
| 3 | **ODOMETRÍA** | Modo HUB **y** pose confirmada por visión | `Turret.computeTurretAngleRad()`, el método de competencia |
| 4 | **BARRIDO** | Nada de lo anterior | Barre buscando, rotando pipelines |

El orden importa, y el nivel 4 es el que hace que todo arranque solo. Al dar
enable la pose todavía no está confirmada, así que la torreta **barre**; en
cuanto encuentra un tag se engancha por visión (nivel 1) y de paso la visión
corrige la pose; a partir de ahí, perder el tag ya no importa, porque el nivel 3
toma el relevo y mantiene el HUB por odometría aunque el tag salga del campo de
visión de la cámara.

La diferencia entre HUB y caza libre no está en la máquina: está en
`DemoState.acceptsTag()`. En HUB sólo acepta los IDs del HUB, así que un tag
suelto que alguien traiga en la mano ni siquiera la distrae. En caza libre acepta
cualquiera. El nivel 3 sólo existe en HUB, porque un tag suelto no está en el
layout del campo y la odometría no tiene forma de saber dónde está.

**La compuerta de pose fresca del nivel 3 es correcta aquí, y no en el chasis.**
Es la asimetría que hay que tener presente al tocar este código: la torreta puede
arrancar sola barriendo, así que exigirle pose confirmada no la bloquea nunca. El
chasis en BOMBER **no** tiene ese camino de arranque, y por eso ahí la misma
compuerta creaba un bloqueo circular (ver sección 4).

> **Regla de mantenimiento.** `DemoTurretCommands.java` **no se toca para
> arreglar BOMBER.** En BOMBER la torreta sólo se congela en cero; todo el
> apuntado de ese modo vive en `DemoDriveCommands`, que es quien mueve el chasis.
> Mezclar las dos cosas fue exactamente lo que rompió el rastreo de STRIKER.

**Por qué la distancia sale de odometría y no de visión.** La distancia 3D de la
cámara llegó a reportar 0.45 m con el robot a varios metros del HUB, lo que
colapsaba el mapa de tiro a su primer punto y hacía que **el hood nunca se
moviera**. En HUB la visión hace lo que mejor hace —apuntar y corregir la pose— y
la distancia sale de la pose. En caza libre no hay alternativa: es visión, con
`maxTrustedVisionDistanceMeters` de filtro.

**Al dar enable, el robot está en HUB con potencia de competencia.** El tiro
suave es la smart feature de "sostén un tag y te llega una pelotita", no el
default.

### Memoria de campo

Perder el tag de vista no rompe el apuntado en HUB: el nivel 3 sigue apuntando
por odometría, y la visión sólo mantiene la pose corregida cuando alcanza a ver
algo.

**Las correcciones de pose se filtran por ID de tag:** sólo se aceptan cuando
*todos* los tags visibles son del HUB. Un tag suelto en la mano de un alumno
nunca teletransporta la odometría.

Si pasan más de `odometryTrustSeconds` (8 s) sin confirmación de visión, el
dashboard lo avisa (`Demo/Sin ver tag s`) y la asistencia de chasis en BOMBER se
desactiva — mejor devolverle el giro al piloto que alinearlo a un HUB imaginario.


### BOMBER

> **Requisito:** el chasis sólo asiste mientras mantengas **APUNTAR (X)** o
> **CARGAR (RT)**. Sin botón, el giro es 100% manual — es a propósito, para que
> el modo no le quite el control al piloto sin que nadie lo haya pedido.

**Torreta:** congelada en cero, lo que deja la cámara rígida respecto al chasis.
**Chasis:** mientras apuntas o cargas, se alinea solo para que la trasera apunte
al HUB.

Con la torreta en cero, ambos cañones **y la cámara** apuntan hacia atrás. Eso es
lo que permite el cambio importante: **el chasis ahora se alinea con visión
directa**, no sólo con odometría. Como la cámara mira exactamente por donde salen
los cañones, `tx` *es* el error de rumbo del chasis — con la misma corrección
vectorial al centro del HUB que usa la torreta.

**Dos fuentes, visión primero:**

| Prioridad | Fuente | Cómo |
|---|---|---|
| 1 | **Visión** | Con la torreta en cero la cámara mira por donde salen los cañones, así que `tx` **es** el error de rumbo del chasis. No necesita saber dónde está el robot. |
| 2 | **Odometría** | El método de competencia: rumbo del robot al HUB, más 180°. |

### El traspaso visión → odometría

Cerca del HUB la visión alinea perfecto. El problema estaba en el momento de
perder el tag: el error se recalculaba desde la **posición estimada** del robot,
y esa posición nunca es exacta — menos con una cámara en torreta cuya
transformada todavía no está medida. El salto entre las dos respuestas hacía que
el chasis pegara un tirón y se quedara persiguiendo un rumbo equivocado.

**La solución es aprender el sesgo.** Mientras la visión funciona, se mide cuánto
se equivoca la odometría:

```
sesgo = errorVisión − errorOdometría
```

y al perder el tag se usa `errorOdometría + sesgo`. El traspaso queda continuo
**por construcción**: en el instante exacto del cambio las dos respuestas son
idénticas, así que el chasis ni se entera. Y de paso se corrige el error
sistemático de la pose, que es lo que hacía que el rumbo por odometría estuviera
mal de entrada.

Dos redes de seguridad más:

- **Slew del rumbo objetivo** (`hubAlignMaxTargetRateRadPerSec = 2.0`). Limita
  qué tan rápido puede moverse el rumbo al que el chasis apunta. Una pose basura
  de un solo ciclo ya no puede mandar al robot a dar la vuelta.
- **Distancia mínima** (`hubAlignMinPoseDistanceMeters = 0.8`). Muy cerca del
  objetivo, centímetros de error de posición se vuelven decenas de grados de
  error de rumbo. Por debajo del umbral se congela el último rumbo bueno.

**En Elastic:** `Demo/Sesgo rumbo deg` te dice cuánto está corrigiendo. Si crece
más de ~20°, la pose está bastante equivocada y vale la pena revisar la
calibración de la cámara (sección 6). `Demo/Fuente apuntado` te dice cuál de las
dos está mandando: `VISION (tag N)`, `ODOMETRIA + sesgo`, `ODOMETRIA (sin
calibrar)`, `RUMBO CONGELADO` o `SIN FUENTE`.

> **El bug anterior.** La asistencia exigía odometría confirmada
> por visión antes de activarse. Eso creaba un bloqueo circular: la asistencia no
> arrancaba sin odometría confirmada → la odometría sólo se confirma cuando la
> cámara ve un tag del HUB → y con la torreta congelada en cero, la cámara sólo
> apunta al HUB **si el chasis ya se alineó**. El modo no podía arrancar nunca.
>
> La torreta en STRIKER no tenía ese problema porque apunta sin verificar nada —
> por eso la torreta funcionaba y el chasis no.
>
> Ahora la compuerta está en `requireFreshOdometryForAssist = false` (el código
> de competencia tampoco verificaba nada), y encima la visión entra como fuente
> primaria, que es lo que rompe el arranque en frío.

> **Por qué se quedaba viendo a la izquierda del HUB:** dos bugs sumados. La
> odometría no se estaba corrigiendo con visión (nunca se supo dónde estaba el
> robot), y el HUB estaba cableado a mano al **azul** — así que en estación roja
> apuntaba a un punto del otro lado del campo. Ahora el HUB se elige por alianza
> (`useAllianceHub`) y la visión tiene prioridad sobre la odometría.

### Smart feature de BOMBER — follow-me

El robot sigue un AprilTag manteniendo distancia. **La torreta no se mueve:**
todo el rastreo lo hace el chasis, igual que el orbit al HUB.

La versión anterior hacía que la torreta rastreara y el chasis persiguiera el
ángulo de la torreta ("la cabeza va primero"). Se veía bien en el papel, pero
arrastraba el sentido de giro del mecanismo —que no se puede deducir del código—
y terminaba girando al revés hasta topar la torreta y obligarla a dar la vuelta
completa.

Ahora, con la torreta congelada en cero, la cámara mira por donde salen los
cañones y el control se reduce a dos números:

```
error de rumbo     = −tx                              → girar el chasis
error de distancia = distancia medida − 1.5 m          → acercarse o alejarse
```

Sólo depende de `tx` y de la distancia. Ni pose 3D, ni convención de yaw del tag,
ni signos que adivinar. Y el rumbo usa el mismo controlador suave que el orbit.

**Envolvente de seguridad:** máx 1.2 m/s · distancia objetivo 1.5 m · piso duro
1.2 m (por debajo sólo puede alejarse) · frena si pierde el target 0.5 s ·
cualquier movimiento del stick del piloto lo cancela al instante · el cañón fijo
queda apagado.

---

## 4.4 Volcado suave (smart feature de BOMBER en modo solo)

**El robot no corrige nada.** Dispara con ambos cañones exactamente hacia donde
está apuntando, con el hood plano y potencia fija. El piloto apunta con el robot;
el código no calcula distancias, ni rumbos, ni objetivos.

Es el modo para mover FUEL de un lado a otro sin ceremonia.

| Qué | Cómo queda |
|---|---|
| Chasis | 100% manual. Sin asistencia de rumbo, ni siquiera con el gatillo. |
| Torreta | Congelada en cero, apuntando hacia atrás con el cañón fijo. |
| Hood | Al ángulo más plano, para que la pelota salga hacia adelante y no en arco. |
| Potencia | Fija, sin mapas. |
| Limelight | No participa en nada. |

**Cómo se usa:** BOMBER → **Back** para activarlo, apuntas el robot a donde
quieres tirar, **RT** para cargar, **LB+RB** para alimentar.

### Constantes

```java
dumpHoodDegrees      = 16.5   // más grados = trayectoria más plana
dumpPowerFraction    = 0.40   // 40% de la velocidad libre del motor
flywheelFreeSpeedRPS = 100.0  // Kraken X60 a 1:1
```

**Sobre el hood:** más grados = más plano, que es el mismo criterio de
`kShooterHoodMap` (los tiros lejanos, que son más planos y rápidos, usan más
offset). 16.5° es el valor más alto que aparece en tu mapa de competencia, así
que es terreno conocido. El recorrido mecánico llega a ~17°; si quieres el máximo
absoluto súbelo con cuidado y escucha que el hood no tope.

**Sobre la potencia — ojo con la referencia.** `0.40` es el 40% de la velocidad
libre del motor, o sea **40 RPS**. Eso queda muy cerca de la velocidad de
competencia (33-45 RPS), así que si se siente muy fuerte, probablemente querías
el 40% de la potencia de competencia, que serían ~18 RPS → `0.18`. Es un solo
número.

---

## 4.5 Disparo en movimiento (shoot while moving)

La pelota sale del robot cargando **la velocidad del robot**. A 2 m/s con un
vuelo de 0.6 s, se desvía 1.2 m del punto al que apuntaste.

El método es el **objetivo virtual**, que es lo que usan los equipos que lo
hacen: en vez de compensar el ángulo con una fórmula aparte, se corre el punto de
mira en dirección contraria a tu velocidad y se apunta ahí normalmente.

```
objetivoVirtual = objetivoReal − velocidad × tiempoDeVuelo
```

Lo elegante es que **resuelve el ángulo Y la distancia de una sola vez**. Si te
mueves **acercándote**, el objetivo virtual queda entre tú y el HUB —o sea, más
cerca— y el mapa pide **menos** potencia, porque la pelota ya lleva tu velocidad
hacia adelante. Alejándote pasa lo contrario. No hay nada que compensar por
separado.

Se itera dos veces porque mover el objetivo cambia la distancia, y la distancia
cambia el tiempo de vuelo.

### Apuntado continuo: por qué la torreta ya no se rezaga

Antes la torreta cerraba el lazo sobre `tx`, así que heredaba los defectos de la
cámara: ~22 FPS y 45 ms de latencia, más una histéresis que congelaba el setpoint
hasta que el error pasaba de 2.5°. Estrafeando a 0.5 m/s a 3 m del HUB, el HUB se
mueve a 9.5°/s respecto al robot y la torreta se quedaba **5-6° atrás**,
moviéndose a brincos. Rezagada hacia el lado del que vienes — que desde afuera se
ve idéntico a una compensación con el signo invertido.

Ahora, en modo HUB con la pose fresca, el setpoint sale de la **odometría** cada
20 ms y la visión sólo aporta un **sesgo** filtrado lento:

```
sesgo    = filtro(ánguloVisión − ánguloOdometría)
setpoint = ánguloOdometría(HUB, sesgo + compensación de movimiento)
```

El seguimiento rápido lo hace la odometría, que no tiene ruido ni latencia; el
sesgo sólo corrige el error sistemático de la pose y de la calibración de la
cámara, que cambia despacio y por eso se filtra fuerte. Es el mismo truco que
`HubAlignment` ya usaba para el chasis.

Efecto secundario que importa: **perder el tag ya no cambia nada**. Antes había un
traspaso visión → odometría; ahora el apuntado siempre salió de la odometría y lo
único que se congela es el sesgo. En el dashboard lo ves como `HUB continuo ·
visión` contra `HUB continuo · odometría (Xs)`.

Para volver al comportamiento anterior: `turretContinuousOdometryAim = false`.

### Cómo llega la corrección a los mecanismos

Esto es lo que más importa entender antes de calibrar, porque es donde está el
truco:

| Consumidor | Qué recibe |
|---|---|
| Hood y flywheel (torreta y cañón fijo) | `distanceMeters` — la distancia al objetivo virtual |
| Ángulo de la torreta | `aimOffsetRad` — **cuánto girar de más**, no a dónde apuntar |
| Rumbo del chasis en BOMBER | el mismo `aimOffsetRad` |

El ángulo va como **delta** y no como setpoint absoluto por una razón concreta:
la torreta apunta con `tx` cuando ve el tag y con la pose cuando no. Si la
compensación viviera sólo en la rama de odometría, la torreta pegaría un brinco
cada vez que el tag aparece o se pierde — una rama compensando y la otra no. Como
delta se le suma a las dos y el traspaso sigue siendo continuo.

Por lo mismo, la histéresis de enganche evalúa el error **con** la compensación
incluida. Si evaluara `tx` a secas, apuntar al centro del tag contaría como
"enganchada" y congelaría el setpoint justo cuando la compensación pide estar
corrido unos grados.

### Procedimiento de tuning

**Arranca apagado a propósito**, porque depende del mapa de tiempo de vuelo que
todavía no está medido.

1. **Mide el tiempo de vuelo.** Graba con el celular a 60 fps desde el costado,
   cuenta los cuadros entre que la pelota sale del shooter y toca el HUB, divide
   entre 60. Repite a 2, 3, 4 y 5 m y llena `kTimeOfFlightMap`.
2. `shootWhileMovingEnabled = true`, `shootWhileMovingGain = 0.3`.
3. **Primero sin disparar.** Apunta al HUB, maneja lateralmente y mira
   `Demo/Compensacion mov deg`: parado debe ser **0 exacto**, y al moverte debe
   crecer proporcional a tu velocidad y hacia el lado al que te mueves.

   En el log hay dos claves y la diferencia importa: `Demo/Shot/AimOffsetDeg` es
   lo que la matemática **calculó**, y `Demo/Turret/AimOffsetDeg` es lo que la
   torreta **aplicó**. En modo HUB deben ser idénticas; si no lo son, algo está
   gateando la corrección. En caza libre la aplicada es 0 a propósito, porque el
   objetivo de la solución de tiro es el HUB y la torreta está mirando otra cosa.
4. Ya con la corrección viéndose sana, dispara moviéndote. Si los tiros se
   corrigen pero se quedan cortos, sube la ganancia de 0.2 en 0.2 hasta 1.0.
5. **Si los tiros se van MÁS al lado en vez de corregirse**, párale. Son dos
   fallas distintas que se ven igual y se arreglan al revés una de la otra, y el
   número que las distingue es `Demo/Turret/AimOffsetDeg` manejando a la
   **derecha** frente al HUB:

   | Lo que ves | Qué pasa | Qué haces |
   |---|---|---|
   | Offset **positivo**, tiros aún a la derecha | El signo está bien, falta corrección — la ganancia arranca en 0.3, o sea 30% de lo teórico | Sube `shootWhileMovingGain` |
   | Offset **negativo** | El signo sí está invertido | `shootWhileMovingAimSign = -1.0` |

   Invertir el signo cuando el problema era la ganancia duplica el error en vez
   de arreglarlo, y desde el otro lado del gimnasio las dos cosas se ven igual de
   mal. Por eso se mira el número antes de tocar la constante.

```java
shootWhileMovingEnabled  = true    // ← el interruptor
shootWhileMovingGain     = 1.0     // 1.0 = compensación teórica completa
shootWhileMovingAimSign  = 1.0     // ← aquí se invierte el signo, y sólo aquí
shootWhileMovingMinSpeed = 0.1     // bajo esto no compensa
shootWhileMovingMaxCompensationMeters = 2.5   // tope en metros
shootWhileMovingMaxAimOffsetRad = 20°         // tope en ángulo
fieldVelocityFilterAlpha = 0.4                // filtro de la velocidad
```

> Los valores de arriba son los que están en el código ahora. Si los cambias en
> `DemoConstants`, este bloque queda mentiroso — el archivo manda.

Los dos topes existen porque uno solo no alcanza: 2.5 m de corrección son ~15° a
9 m de distancia pero más de 60° a 2 m. El tope angular es el que evita que la
torreta se vaya a medio campo cuando estás pegado al HUB.

### Si al prender el interruptor todo se pone raro

La velocidad viene de los estados de los módulos y trae ruido de encoders y
patinado. Ese ruido llega al hood, al flywheel y a la torreta convertido en
comandos nuevos cada 20 ms, y cada comando nuevo **reinicia el perfil de Motion
Magic**: el mecanismo tiembla en su lugar, chupa corriente y en el peor de los
casos te tira el voltaje lo suficiente para que la Limelight deje de dar poses —
y a los 8 s (`odometryTrustSeconds`) la torreta se queda sin odometría y se pone
a barrer.

Contra eso hay tres defensas, y si algo se pone raro es lo primero que hay que
revisar:

- `fieldVelocityFilterAlpha` — filtro pasa-bajas de la velocidad. Bájalo si la
  corrección tiembla.
- `hoodSetpointDeadbandDeg` y `flywheelSetpointDeadbandRPS` — no re-comandar
  cambios insignificantes. Súbelos si escuchas al hood zumbar.
- La distancia ahora se **recorta** al rango `[minShotDistanceMeters,
  maxShotDistanceMeters]` en vez de saltar a 3 m cuando se sale. El salto viejo
  se cruzaba seguido con la compensación encendida, porque manejar hacia el HUB
  acerca el punto de mira.

El modo solo hace esto mucho más fácil de probar: manejas y disparas tú mismo sin
depender de nadie.

---

## 4.6 Nota sobre las AprilTags del HUB

Cada cara del HUB tiene dos tags: una centrada y otra a su izquierda.

| | Rojas | Azules |
|---|---|---|
| Centrales | 2, 4, 8, 10 | 18, 20, 24, 26 |
| Izquierdas | 3, 5, 9, 11 | 19, 21, 25, 27 |

Con el apuntado de vuelta en odometría, estos IDs ya **no** se usan para apuntar
— sólo para filtrar qué tags tienen permiso de corregir la pose.

> El código de apuntado vectorial (seleccionar la tag central con `priorityid` y
> corregir del centro del tag al centro del HUB) sigue en
> `commands/demo/HubAiming.java`, marcado como no usado. Si algún día quieren
> refinar el apuntado a corta distancia con visión, ahí está la base.

---

## 5. Limelight 2 / 2+ — calibración para largo alcance en salones

Volvimos a la cámara de siempre (`limelight-fixed`). En el código sólo hubo que
poner `DemoConstants.isLimelight4 = false`, que apaga las dos claves que sólo
existen en la LL4. Todo lo demás —pose de cámara dinámica, `priorityid`,
latencia real— funciona igual en la LL2.

### 5.0 Requisito previo: firmware

**La LL2/2+ necesita LimelightOS 2024 o más reciente.** Sin eso no tiene
AprilTags, ni `priorityid`, ni `camerapose_robotspace_set` — y sin esa última, la
cámara en la torreta no puede reportar poses correctas.

Buena noticia: aunque el firmware sea viejo, **el rastreo de STRIKER sigue
funcionando**, porque el apuntado cierra el lazo sobre `tx` y `tx` no depende de
ninguna transformada. Lo que se degrada es la distancia y el follow-me.

Revísalo en la web UI, esquina inferior. Si dice 2023 o anterior, actualiza antes
de perder una tarde depurando.

### 5.1 Lo que realmente limita el alcance

Con qué contamos: sensor a color con **rolling shutter**, lente de **enfoque
fijo**, sin calibración ChArUco, y tags oficiales de **16.5 cm** que no podemos
hacer más grandes.

Expectativa realista y honesta:

| Qué | Alcance |
|---|---|
| Detección 2D (`tx`, apuntar) | ~5-6 m a resolución completa |
| Pose 3D (`targetpose_robotspace`, distancia) | ~3-4 m |

**Esa diferencia es el punto clave que casi nadie considera.** Para apuntar basta
con ver el tag; para resolver su pose 3D hay que medir con precisión sus cuatro
esquinas, y eso se degrada mucho antes. Por eso
`DemoConstants.maxTrustedVisionDistanceMeters` está en 4.0: pasando eso, el
código deja de creerle a la distancia de la cámara y usa odometría o el valor de
respaldo. Sin ese límite, el mapa de tiro estaría saltando entre RPS distintos
cada frame.

### 5.2 Las dos perillas que más alcance dan

**a) Resolución 960x720.** No 640x480. Es el factor número uno y la LL2 muchas
veces viene configurada en la resolución baja.

**b) Downscaling / decimate = 1.0.** No 2.0, no "auto". El downscaling divide la
resolución efectiva del detector: en 2x tienes literalmente la mitad del alcance.
Súbelo sólo si necesitas FPS.

Entre las dos, pasar de "640x480 + decimate 2" a "960x720 + decimate 1" puede
**duplicar** la distancia a la que engancha.

El costo: 960x720 corre a ~22 FPS en vez de 90. Para una demo está perfectamente
bien, pero **sí afecta al apuntado**, y por eso subí
`turretExtraLatencySeconds` de 0.020 a **0.045**: a 22 FPS, cada frame tiene en
promedio medio periodo (22 ms) de antigüedad que `tl` no reporta. Si algún día
bajan a 640x480, bajen también esa constante a ~0.020.

También agregué detección de muestra nueva: el código corre a 50 Hz y la cámara a
22, así que dos de cada tres lecturas son el mismo frame repetido. Volver a
procesarlo aplicaría la compensación de latencia sobre un ángulo de torreta que
sí cambió, y el estimado se iría derivando solo.

### 5.3 Enfoque

La LL2/2+ tiene **enfoque fijo de fábrica**. No hay nada que ajustar, pero sí hay
que verificar: mira el stream a la distancia de trabajo. Si el tag se ve borroso,
ese es el techo de todo lo demás y ninguna configuración lo va a arreglar.

> Existe el truco de despegar el lente y reenfocarlo. Es irreversible y arruina
> cámaras. No lo hagan a dos días de una demo.

### 5.4 Salón iluminado: aquí tenemos ventaja

Un salón con luz blanca intensa suena a problema y en realidad es lo mejor que le
puede pasar a una cámara de rolling shutter montada en una torreta que gira:
**mucha luz permite exposición muy corta, y exposición corta mata el motion
blur.**

Configuración base:

- **Exposición: lo más baja posible** que aún dé una imagen clara
- **Gain (ganancia): baja.** La ganancia alta mete ruido y el ruido rompe la
  detección de esquinas más rápido que la falta de luz
- **LEDs: apagados.** Los LEDs verdes de la LL2 son para retrorreflectivo, no
  hacen nada por los AprilTags y en un cuarto ya iluminado sólo agregan reflejo
- **White balance: fija.** La LL2 es a color y el auto-WB anda persiguiendo el
  blanco bajo LED
- **Black level: súbelo un poco** para que los cuadros negros sigan leyéndose
  negros con tanta luz ambiental rebotando

**Síntoma de sobreexposición:** el borde blanco del tag se "derrama" sobre el
negro, y la detección va y viene aunque el tag se vea grande y nítido. Si
detecta lejos pero falla cerca, es esto.

### 5.5 El problema real: techo bajo y reflejo

Éste va a ser su enemigo, más que la distancia. Lámparas potentes a poca altura
pegan al tag en ángulo cerrado y el reflejo entra directo al lente. El detector
ve un rectángulo blanco brillante y no encuentra el patrón.

Tres soluciones, en orden de efectividad:

1. **Inclina el tag 10-15° hacia adelante** (la parte de arriba hacia la cámara).
   El reflejo del techo se va al piso en vez de al lente. Es gratis y resuelve la
   mayoría de los casos.
2. **Nada de laminado brillante.** Si van a proteger los tags, laminado **mate**.
   Un tag laminado brillante bajo luz de techo es prácticamente indetectable.
3. **No pares el robot justo debajo de una lámpara** y no pongas el tag
   directamente bajo una. Un paso de lado suele arreglar todo.

### 5.6 Parpadeo de las lámparas

Las lámparas fluorescentes y muchos paneles LED baratos parpadean al doble de la
frecuencia de la red: en México, **120 Hz, o sea un ciclo cada 8.33 ms**.

Con rolling shutter y exposición corta, eso aparece como **bandas horizontales**
que se mueven por la imagen. Si una banda oscura cruza el tag justo en el frame
que importa, se pierde la detección — y el síntoma es "a veces engancha y a veces
no, sin razón aparente".

**La prueba:** mira el stream con el robot quieto. Si la imagen pulsa o tiene
franjas que suben, es parpadeo.

**La solución:** poner la exposición en un múltiplo del periodo de parpadeo, para
que cada frame integre un ciclo completo. En Limelight el valor de exposición
está en centésimas de milisegundo, así que 8.33 ms ≈ **833**.

El costo es una exposición larga, que trae de vuelta el blur. Por eso es el
pipeline 2 y no el default: úsenlo sólo en los salones donde el parpadeo sea
visible.

### 5.7 Montaje del tag

Cosas que parecen menores y no lo son:

- **A la altura de la cámara.** Un tag visto desde muy abajo o muy arriba se ve
  como trapecio, y eso destruye la pose 3D antes que la distancia.
- **Plano y rígido.** Cartón pluma o coroplast, no una hoja pegada con cinta a
  una carpeta. Un tag ondulado pierde esquinas.
- **De frente.** Más de ~45° de ángulo y la pose 3D deja de ser confiable aunque
  el tag siga detectándose.

### 5.8 Los tres pipelines

Configúralos como **AprilTag**, familia **36h11**, en los índices 0, 1 y 2. El
código los rota cada 2.5 s mientras busca.

| # | Nombre | Resolución | Decimate | Exposición | Para qué |
|---|---|---|---|---|---|
| **0** | Largo alcance | 960x720 | 1.0 | Baja | Default. Apuntar al HUB y cazar tags lejanos. ~22 FPS. |
| **1** | Cerca y rápido | 640x480 | 1.0 | Baja | Follow-me y seguimiento cercano. ~90 FPS, mucho menos latencia. |
| **2** | Anti-parpadeo | 960x720 | 1.0 | ~833 (8.33 ms) | Sólo para salones donde se vean bandas. |

En los tres: LEDs apagados, gain bajo, white balance fija.

> **Truco:** el follow-me se siente notablemente mejor en el pipeline 1, porque a
> 90 FPS la latencia baja y la torreta responde más rápido. Si les gusta cómo
> queda, pueden hacer que el código lo seleccione solo cuando
> `DemoState.isFollowing()` — es un `if` de una línea en `DemoTurretCommands`.
> No lo dejé puesto para no meter código sin probar.

### 5.9 Lo del montaje en torreta que sigue aplicando

**Pose de cámara en ceros en la web UI.** El código la publica cada ciclo con el
ángulo actual de la torreta; si además dejan valores en la UI, se suman y todo
queda corrido.

### 5.10 Si algún día conectan bien la Limelight 4

El soporte sigue en el código, apagado. Para reactivarlo:

1. `DemoConstants.isLimelight4 = true`
2. `DemoConstants.turretCameraName = "limelight"` (o el hostname que le pongan)
3. `turretExtraLatencySeconds` de vuelta a ~0.020 — la LL4 va a 120 FPS
4. `maxTrustedVisionDistanceMeters` se puede subir a ~6.0: sensor global shutter,
   mayor resolución y calibración ChArUco integrada mejoran mucho la pose 3D

Dos advertencias para ese día:

- **La LL4 ya no soporta POE.** Necesita dos cables 18-20 AWG al puerto
  Weidmuller desde el PDP/PDH con breaker de 5 o 10 A. Eso es exactamente lo que
  no dio tiempo ahora.
- **Modo de IMU = 0, obligatorio.** La LL4 trae IMU interna, pero está
  atornillada a la cámara y la cámara a la torreta: cuando la torreta gira 90°,
  la IMU cree que el robot giró 90°. Cualquier otro modo le daría a MegaTag2 un
  yaw que no corresponde al chasis y rompería la localización de una forma sutil
  y difícil de diagnosticar. El código ya lo fuerza cuando `isLimelight4` está en
  true.


## 6. Calibrar la geometría de la cámara

Al mover la cámara a la torreta, la transformada robot→cámara dejó de ser
constante. El código la recalcula cada ciclo, pero necesita dos medidas.

```java
// DemoConstants.java
public static final Translation3d robotToTurretPivot =
    new Translation3d(-0.165, -0.170, 0.210);

public static final Transform3d turretPivotToCamera = new Transform3d(
    new Translation3d(0.120, 0.0, 0.080),
    new Rotation3d(0.0, Math.toRadians(-15.0), 0.0));
```

Convención WPILib: **+X adelante, +Y izquierda, +Z arriba.** Origen del robot =
centro del frame a nivel de piso.

### Paso 1 — Medir con cinta

Robot en el piso, torreta en su cero mecánico:

1. Marca el centro del frame (intersección de las diagonales del bumper)
2. **`robotToTurretPivot`** — del centro del frame al eje de giro de la torreta
   (X adelante+, Y izquierda+, Z = altura del eje sobre el piso)
3. **`turretPivotToCamera`** — del eje de giro al lente, en el marco de la
   torreta (+X = hacia donde apunta el cañón). El pitch **negativo mira hacia
   arriba**

### Paso 2 — La prueba de los 2 metros

1. Torreta en cero, robot deshabilitado
2. AprilTag **exactamente a 2.00 m** del centro del robot, de frente
3. En AdvantageScope grafica `Vision/Camera0/targetPoseRobotSpace`
4. Debe leer **X ≈ 2.00, Y ≈ 0.00, Z ≈ altura del tag**

| Lectura mala | Qué ajustar |
|---|---|
| X corrido | Suma de las dos X |
| Y corrido | Suma de las dos Y |
| Z corrido | Z del lente, o el pitch |

### Paso 3 — La prueba decisiva

Deja el tag donde está y **gira la torreta 45° a mano**.

`targetPoseRobotSpace` **no debe cambiar**. El tag no se movió, así que su pose
relativa al *robot* tiene que seguir igual aunque la cámara haya girado.

| Síntoma | Causa |
|---|---|
| La pose gira al revés | `limelightInvertSideAxis = true` |
| La pose se voltea 180° | Revisar `TurretConstants.turretZeroOffsetRad` (debe ser π) |
| La pose se desplaza | `robotToTurretPivot` no está sobre el eje real de giro |

### Paso 4 — Signo de `tx`

Habilita en STRIKER, mantén X, y pon un tag claramente **a la derecha**. La
torreta debe girar **hacia** el tag. Si se aleja: `turretTxSign = -1.0`.

### Paso 5 — Yaw del tag (sólo para follow-me)

Tag de frente, a ~3 m. Activa BOMBER → Back. El robot debe estacionarse
**enfrente** del tag a 1.5 m.

| Síntoma | Ajuste |
|---|---|
| Se va detrás del tag | `followTagYawOffsetRad` entre `Math.PI` y `0.0` |
| Se va de lado al girar el tag | `limelightInvertSideAxis` |
| Sigue raro | `followOrbitStrength = 0.0` — pierdes el orbitado, el seguimiento a distancia sigue funcionando |

---

## 7. El temblor de la torreta: la histéresis de enganche

Después de la compensación de latencia, la torreta seguía temblando al quedarse
viendo su objetivo. Ese síntoma —temblor **con el objetivo quieto**— no se
arregla con tuning, porque no es un problema de ganancia.

### Por qué temblaba

Por buena que sea la ganancia, `tx` nunca es exactamente cero: oscila un par de
décimas de grado por ruido de píxel. Con cualquier lazo continuo esas décimas se
convierten en comandos, **cada comando reinicia el perfil de Motion Magic**, y el
mecanismo tiembla eternamente persiguiendo ruido que no significa nada.

### La solución no es más tuning: es dejar de comandar

```java
turretLockThresholdRad   = 1.0°   // bajo esto, se engancha y CONGELA
turretUnlockThresholdRad = 2.5°   // sobre esto, se desengancha
```

Cuando el error cae bajo el umbral de enganche, la torreta se declara enganchada
y **congela su setpoint**. No vuelve a moverse hasta que el error supere el
umbral de desenganche, que es notablemente mayor — así el ruido nunca alcanza a
desengancharla.

La diferencia entre ambos umbrales **es** la histéresis. Si los pones muy
parecidos, el ruido hace que enganche y desenganche todo el tiempo y vuelve el
temblor.

Efecto visible: la torreta se mueve, se planta, y se queda absolutamente quieta
hasta que el objetivo se mueve de verdad.

### Ganancias propias del demo

Las de competencia traen `kI = 0.1`. Un término integral en un mecanismo
posicional con Motion Magic acumula error mientras el perfil está en camino y
luego sobrepasa — es una fuente clásica de cacería de baja frecuencia. Para
*seguir* un objetivo suavemente no hace falta integral: el feedforward de Motion
Magic ya se encarga del error estático.

```java
useDemoTurretGains = true       // false vuelve exactamente a competencia
turretDemoKp = 38.0             // era 50
turretDemoKi = 0.0              // era 0.1  ← el importante
turretDemoKd = 3.5              // era 2.0
turretDemoCruiseRotPerSec   = 1.2   // era 2.0
turretDemoAccelRotPerSecSec = 2.5   // era 5.0
```

Los perfiles de Motion Magic también bajaron: competencia está tuneado para
*llegar rápido* a un setpoint, y para *seguir* conviene lo contrario — perfiles
lentos que no alcancen a saturar entre comando y comando.

### Y encima de eso

Sigue vigente todo lo anterior: compensación de latencia (el `tx` que leemos
describe dónde estaba el target hace ~40 ms), detección de muestra nueva (la
cámara va a 22 FPS y el código a 50 Hz), filtro sobre la estimación absoluta, y
límite de velocidad del setpoint.

| Síntoma | Ajuste |
|---|---|
| Sigue temblando enganchada | Sube `turretLockThresholdRad` a 1.5° |
| Engancha y desengancha seguido | Separa más los dos umbrales |
| Tarda en reaccionar cuando el alumno se mueve | Baja `turretUnlockThresholdRad` a 2.0° |
| Se pasa al girar rápido | Sube `turretExtraLatencySeconds` |
| Persigue ruido de píxel | Baja `turretTrackFilterAlpha` a 0.08 |

---

## 8. Indicadores en Elastic

### Importar el layout

Ya está desplegado con el código. En Elastic:

**File → Load Layout From Robot** (o `Ctrl + D`) → elige `elastic-layout.json` →
modo **Overwrite**.

Vienen dos pestañas:

- **Demo** — lo que ve el operador durante la actividad
- **Diagnostico** — gráficas para tunear el tracking

> Si la importación falla por diferencia de versión de Elastic, arma la pestaña
> a mano en tres minutos: todos los topics están abajo, y arrastrarlos desde el
> árbol de NetworkTables crea el widget correcto solo.

### Los tres indicadores que importan

| Widget | Topic | Significa |
|---|---|---|
| **LISTO PARA TIRAR** | `Demo/LISTO PARA TIRAR` | Apuntada **y** acelerada **y** cargando. Verde = alimenta y entra. |
| **Objetivo a la vista** | `Demo/Objetivo a la vista` | La cámara ve un tag válido |
| **Torreta apuntada** | `Demo/Torreta apuntada` | Error dentro de tolerancia 4 ciclos seguidos |

### Todos los topics

Todo bajo `/SmartDashboard/Demo/`:

**Texto**

```
Estado              línea completa: "STRIKER · APUNTADA · tag 7 · 2.4 m · CARGANDO SUAVE"
Modo                STRIKER / BOMBER
Feature             Caza global / Tag fijado: N / Orbitar HUB / Follow-me
Torreta             INACTIVA / BUSCANDO / ENGANCHADA / APUNTADA / PERDIDA (esperando)
Potencia de tiro    SUAVE / COMPETENCIA / —
```

**Booleanos**

```
LISTO PARA TIRAR      Objetivo a la vista     Torreta apuntada
Flywheels listos      Limelight OK            Apuntando
Cargando              Chasis asistido         Modo precision
Siguiendo target      Follow enganchado
```

**Números**

```
Tag ID          Distancia m         Error torreta deg
Velocidad pct   Pipeline            Latencia ms
Tag lockeado    Follow distancia m
```

Más `Field` (posición del robot), `Auto Choices` y `FMSInfo`.

---

## 9. El chasis: qué estaba mal y qué se hizo

Ésta fue la queja principal de las pruebas y tenían toda la razón. El problema
era **conceptual**, no de tuning: lo que estaba escrito no era un filtro, era un
controlador.

### El bug

La primera versión usaba un perfil de **tiempo mínimo (bang-bang)**: calculaba la
distancia de frenado y comandaba **aceleración máxima** siempre que hubiera
cualquier error, hasta que tocaba empezar a frenar.

Eso explica, una por una, todas las quejas:

| Lo que sintieron | Por qué pasaba |
|---|---|
| "Movemos tantito el joystick y acelera brutalmente" | Un stick al 5% generaba un error de 0.05, y el controlador respondía con la misma aceleración máxima que habría usado para ir al 100%. No existía el concepto de "movimiento pequeño". |
| "Se avanza cañón" / "tiene lag" | La tasa interna tenía inercia propia (limitada por jerk), así que al centrar el stick el comando seguía creciendo unos ciclos antes de poder revertirse. |
| "Parece que se maneja solo" | Porque literalmente lo hacía: era un servo persiguiendo un setpoint con dinámica propia, no un filtro sobre la intención del driver. |
| "Como que tiene drift" | El frenado ante inversión: si cambiabas de dirección más de 100°, el filtro decidía frenar a cero **conservando el rumbo viejo** antes de hacerte caso. |

### Lo que se quitó

1. **El perfil bang-bang.** Reemplazado por un limitador de tasa real.
2. **El frenado ante inversión.** Ahora una inversión se maneja sola: el vector
   de error apunta hacia atrás y el comando pasa por cero de forma natural.
3. **El límite de aceleración lateral (arc limiting).** Físicamente correcto,
   pero era justo lo que hacía sentir que el robot buscaba una posición en vez de
   obedecer. Y era redundante: las llantas ya imponen ese límite solas. Si
   derrapa, se baja la aceleración, no se le agrega una regla más.

### Cómo funciona ahora

```
error      = deseado − actual                    (vector)
tasaPedida = recortarNorma(error / dt, límite)
tasa       = pasoBajo(tasa → tasaPedida, tau)
paso       = tasa · dt, nunca mayor que el error
actual    += paso
```

La propiedad clave: **si el driver pide un cambio que cabe dentro del límite, ese
cambio pasa completo y sin retardo.** Los límites sólo entran a trabajar cuando
se le pide al hardware algo que no puede dar. Eso es lo que hace que se sienta en
control.

Y el "nunca mayor que el error" garantiza que soltar el stick detiene el robot:
no se puede rebasar el objetivo, así que no hay coleo.

Además es **isotrópico** — se recorta la norma del vector, no cada eje por
separado. Recortar por eje haría que una diagonal acelerara √2 veces más rápido
que un movimiento recto.

### Constantes

```java
maxSpeedFraction        = 0.50   // ← bajado 30% respecto a la prueba
creepSpeedFraction      = 0.25
translationMaxAccel     = 2.6    // 0 → 100% en ~0.38 s
translationMaxDecel     = 3.2    // 100% → 0 en ~0.31 s
translationSmoothingTau = 0.15   // ← +50% de suavizado
rotationSmoothingTau    = 0.12
joystickExponent        = 1.4
joystickDeadband        = 0.08
```

> Estos son los valores que pediste después de la prueba: 50% más de suavizado
> (tau 0.10 → 0.15) y 30% menos de velocidad (0.70 → 0.50 de la velocidad libre,
> o sea ~3.0 m/s). Si después de rodarlo con novatos quieres devolverle punch,
> las dos perillas están arriba y son independientes: la velocidad no afecta el
> tacto y el tacto no afecta la velocidad.

Los números son **más agresivos** que antes a propósito. Con un bang-bang,
valores "suaves" daban un robot brusco. Con un limitador, valores más altos se
sienten más directos, no más violentos.

`translationSmoothingTau` es **la** perilla del tacto:

| Valor | Cómo se siente |
|---|---|
| 0.00 | Trapecio puro. Máxima respuesta, se siente el tirón mecánico. |
| 0.10 | Suave, sin retardo perceptible. |
| 0.15 | **Default actual.** Amigable para novatos. |
| 0.20 | Muy suave, el driver empieza a notar lag. |
| 0.30+ | Como manejar en gelatina. No. |

### Síntoma → constante

| Lo que sientes | Ajuste |
|---|---|
| "Le falta punch al arrancar" | Sube `translationMaxAccel` |
| "Se sigue de largo al soltar" | Sube `translationMaxDecel` |
| "Todavía se siente el tirón" | Sube `translationSmoothingTau` a 0.15 |
| "Se siente con lag" | Baja `translationSmoothingTau` a 0.05 |
| "Nervioso cerca del centro" | Sube `joystickExponent` a 1.7 |
| "Perezoso, no responde" | Baja `joystickExponent` a 1.0 |

Para verlo en AdvantageScope: grafica `Demo/Drive/RawMagnitude` contra
`Demo/Drive/CommandMagnitude`. **Las dos curvas deben verse casi encimadas**, con
la del comando siguiendo a la del stick con un retraso apenas visible. Si la del
comando se dispara y sobrepasa, algo sigue mal.

### El PID de rumbo de BOMBER

Mismo diagnóstico, dos causas:

1. Pasaba por el mismo limitador roto, así que heredaba toda la brusquedad.
2. Usaba `atSetpoint()` para cortar la salida de golpe al llegar a la tolerancia.
   Ese corte es una discontinuidad: el chasis pasa de corregir a no corregir en un
   ciclo, se pasa un poco, vuelve a corregir de golpe. Ése es el "glitchy y medio
   agresivo".

Ahora hay una **compuerta suave**: la salida se escala de 0 a 1 a lo largo de una
rampa entre `headingToleranceRad` (1.5°) y `+ headingSoftZoneRad` (4°). Sin
escalón, sin buzz.

Y las ganancias se re-tunearon para amortiguar más:

```java
headingKp                = 2.4   // era 4.0
headingKd                = 0.35  // era 0.15
headingMaxOmegaRadPerSec = 2.5   // era 3.5
```

---

## 9.5 Mapa de tiro suave — tunéalo por puntos

Mismo formato que `ShooterConstants`: un punto por distancia, con su potencia y
su ángulo de hood. En `DemoConstants.java`:

```java
kGentleFlywheelMap.put(1.0, 20.0);   kGentleHoodMap.put(1.0, 10.0);
kGentleFlywheelMap.put(2.0, 24.0);   kGentleHoodMap.put(2.0, 13.0);
kGentleFlywheelMap.put(3.0, 28.0);   kGentleHoodMap.put(3.0, 15.0);
kGentleFlywheelMap.put(4.0, 31.0);   kGentleHoodMap.put(4.0, 16.0);
kGentleFlywheelMap.put(5.0, 34.0);   kGentleHoodMap.put(5.0, 16.5);

gentleMaxFlywheelRPS = 34.0;   // tope duro de seguridad
```

Agrega, quita o mueve puntos libremente: entre puntos se interpola lineal y fuera
del rango se hace clamp, igual que en competencia.

**Los valores subieron.** Estaban en 12-22 RPS (~40% de competencia) y la pelota
apenas salía del robot. Ahora están en 20-34 (~65%). Si sigue corta, sube los RPS
de 2 en 2; si llega muy fuerte, bájalos. El hood se mide en grados de offset
desde reposo, igual que `kShooterHoodMap`: más grados = más arco.

`gentleMaxFlywheelRPS` es la red de seguridad — aunque alguien edite mal el mapa,
ningún camino de código puede exceder ese tope mientras el robot le esté
apuntando a un AprilTag suelto.

---

## 10. Intake: agitación y límites

> Nada de esta sección cambió en el rollback. La sincronía intake + feeder, la
> agitación con pre-roll y el remate por corriente se quedaron tal cual.

### 10.1 Dónde se tunean los límites  ← LO QUE PREGUNTASTE

**Todo está en `constants/IntakeConstants.java`**, en radianes del extensor.

Antes los soft limits estaban escritos a mano dentro de `IntakeIOTalonFX`, en
**dos** lugares distintos (el constructor y `setSoftwareLimit`), y encima no
coincidían con las constantes. Eso ya se arregló: el IO lee de aquí.

```java
extendedRotation          = 20 rad   // objetivo al extender  ← súbelo si le falta salida
extendedRotationReversed  = 3.5 rad  // objetivo al retraer
extensorForwardLimitRad   = 21.5     // soft limit hacia afuera
extensorReverseLimitRad   = 2.0      // soft limit hacia adentro
```

Regla: `extensorForwardLimitRad` **tiene que ser mayor** que `extendedRotation`,
si no el mecanismo nunca llega a la posición comandada. Si le subes a
`extendedRotation`, súbele también al soft limit.

### 10.2 Límite por pico de corriente

Sí se puede, y es exactamente la solución correcta a "le falta un poquito".
Ya está implementado y va en el botón **A**.

**Cómo funciona:**

1. **Fase rápida** — lazo cerrado a `extendedRotation`. Recorre casi todo el
   camino en el menor tiempo posible.
2. **Remate** — apaga los soft limits y empuja hacia afuera en lazo abierto a
   `stallHomingSpeed` (12%). Despacio a propósito: el objetivo es *sentir* el
   tope, no golpearlo.
3. **Detección** — cuando la corriente de estator supera `stallCurrentAmps`
   sostenida por `stallDebounceSeconds`, es el tope. El debounce existe porque
   el pico de arranque del motor se vería igual que un tope.
4. **Retroceso** — se despega `stallBackoffRad` y se queda ahí en lazo cerrado.
   Dejar el mecanismo forzando su propio tope calienta el motor y desgasta el
   rack.

La gran ventaja es que **no depende de calibración**: el tope mecánico está
donde está, y el robot lo encuentra solo cada vez. Si el encoder arrancó en otro
lado o el ratio del gearbox tiene división entera, da igual.

```java
useStallHoming        = true
stallHomingSpeed      = 0.12   // duty del empuje final
stallCurrentAmps      = 18.0   // umbral de tope       ← el que hay que medir
stallDebounceSeconds  = 0.12
stallTimeoutSeconds   = 1.2
stallBackoffRad       = 0.3
```

**Cómo medir `stallCurrentAmps`:** extiende el intake al aire y mira
`Intake/extensorCurrentAmps` en AdvantageScope; luego bloquéalo con la mano y
mira el pico. Pon el valor a la mitad entre ambos. Tiene que quedar cómodamente
arriba de la corriente de movimiento libre y cómodamente abajo del límite de
estator (40 A).

Los soft limits **siempre** se restauran al soltar el botón, sin importar cómo
termine el comando. Si se quedaran apagados, el siguiente jog manual podría
destrozar el rack.

Para volver al comportamiento de sólo posición: `useStallHoming = false`.

### 10.3 Agitación

Ahora tiene dos fases, como pediste:

**Fase 1 — Pre-roll (0.5 s).** Sólo gira el rodillo, sin mover el extensor. Es
para las pelotas que quedaron a medio camino en la rampa: si empiezas a sacudir
de inmediato, esas pelotas salen disparadas hacia afuera en vez de terminar de
entrar.

**Fase 2 — Sacudida con envolvente decreciente.**

```
envolvente(t) = e^(−t / 2.5s)
setpoint(t)   = centro(t) + amplitud(t) · sin(2π · 1.2Hz · t)
```

| Tiempo | Comportamiento |
|---|---|
| 0 – 0.5 s | Sólo rodillo. Las pelotas rezagadas terminan de entrar. |
| 0.5 s | Arranca la sacudida, oscilando entre 6 y 16 rad. |
| 3 s | Entre 4.3 y 7 rad. Ya está cerrando. |
| 8 s | Estático en 4 rad. Caja cerrada, pelotas asentadas. |

El rodillo sigue girando hacia adentro todo el tiempo para barrer de vuelta lo
que la sacudida empuje hacia afuera.

Se activa sola al mantener LB o RB, y a mano con POV ↓.

```java
agitatePrerollSeconds = 0.5   // ← la fase nueva
agitateStartCenterRad = 11.0
agitateEndCenterRad   = 4.0
agitateAmplitudeRad   = 5.0
agitateFrequencyHz    = 1.2
agitateDecaySeconds   = 2.5
agitateRollerSpeed    = 0.25
```

> Las posiciones dependen de `IntakeConstants.extensorGearRatio`, que vale 8.0
> por división entera cuando debería ser 9.0. Están calibradas contra el ratio
> **actual**. Si corrigen esa constante, reescalen estos valores igual.

---

## 11. Checklist antes de salir

**En el taller**

- [ ] `RobotConstants.isDemoMode = true`
- [ ] `DemoConstants.isLimelight4 = false` y `turretCameraName = "limelight-fixed"`
- [ ] LimelightOS 2024 o más reciente (sin eso no hay AprilTags ni pose dinámica)
- [ ] Pipeline 0 en **960x720 con decimate 1.0** — es lo que da el alcance
- [ ] Pipelines 0, 1, 2 configurados (largo alcance / rápido / anti-parpadeo)
- [ ] LEDs apagados, gain bajo, white balance fija
- [ ] Imagen nítida en el stream a la distancia de trabajo (enfoque es fijo)
- [ ] Pose de cámara en **ceros** en la web UI
- [ ] Calibración geométrica hecha (sección 6, pasos 1 a 4)
- [ ] `trustVisionPoseInDemo = false` salvo que lleves HUB en posición medida
- [ ] Prueba de inactividad: **habilita sin tocar nada — la torreta no debe moverse**
- [ ] Prueba de enganche: mantén X, mete un tag, verifica que se prende y se asienta
- [ ] Follow-me probado en espacio abierto **antes** de hacerlo con alumnos
- [ ] Layout de Elastic cargado (`Ctrl + D`)
- [ ] Memoria USB montada para los logs

**En el sitio**

- [ ] Apunta el robot hacia "adelante" y presiona **B**
- [ ] Delimita perímetro antes de activar el follow-me
- [ ] Un mentor con el control del piloto a la mano (el stick cancela el follow)
- [ ] Prueba de disparo suave contra una pared antes de aventarle una pelota a alguien

---

## 12. Diagnóstico

| Síntoma | Dónde mirar |
|---|---|
| La torreta no hace nada | Correcto si no mantienes X o RT. |
| La torreta se estaciona a un lado y no vuelve | La pose mandaba el objetivo fuera de su alcance y `computeTurretAngleRad` recortaba al tope. **Arreglado**: ahora mantiene el último apuntado bueno. Estado: `HUB · pose fuera de rango` |
| La torreta apunta a un HUB imaginario | Ya no depende sólo de la pose. Mira `Demo/Sesgo torreta deg`: si es grande, la pose está corrida |
| La torreta tiembla enganchada | Sube `turretLockThresholdRad`. Ver sección 7. |
| BOMBER se vuelve loco al perder el tag | **Arreglado.** El traspaso visión→odometría era discontinuo. Ahora se aprende el sesgo entre ambas. Mira `Demo/Sesgo rumbo deg`. |
| BOMBER no gira el chasis | **Arreglado.** La asistencia exigía odometría confirmada, y eso era un bloqueo circular. Ver sección 4. |
| En BOMBER un hood se ajusta y el otro no | **Arreglado.** Los dos cañones calculaban su distancia por separado y con criterios distintos. Ahora comparten `ShotSolution`. |
| El hood no se mueve al cargar | Revisa `Demo/Distancia m`: si marca menos de 1 m con el robot lejos del HUB, la odometría no está inicializada. Presiona B en modo FIELD y deja que vea un tag del HUB. |
| El tiro suave sale corto | Sube los RPS en `kGentleFlywheelMap`. Sección 9.5. |
| El frente se me mueve solo | Estás en modo FIELD, donde la referencia es el campo. Presiona X para pasar a DRIVER y fija tu frente con B. |
| BOMBER no alinea | `Demo/Fuente apuntado` en NINGUNA = odometría sin confirmar. Necesita ver un tag del HUB primero. |
| El chasis se siente raro / con lag | Grafica `Demo/Drive/RawMagnitude` contra `CommandMagnitude`. Deben verse casi encimadas. Si el comando se dispara, revisa que tengas la versión nueva del filtro |
| La torreta brinca entre dos posiciones frente al HUB | Está saltando entre la tag central y la izquierda. Verifica que `useLimelightPriorityId = true` y que el firmware soporte `priorityid` |
| Apunta al tag y no al centro del HUB | `hubFaceToCenterMeters` sin medir. Grafica `Demo/Hub/AimCorrectionDeg`: de frente ~0, de lado debe crecer |
| El follow-me gira al revés | Indicador `Demo/REVISAR signo follow`. Cambia `followBodySign` de signo |
| El intake no sale completo | Sube `IntakeConstants.extendedRotation` **y** `extensorForwardLimitRad`, o deja que el remate por corriente lo resuelva |
| El remate por corriente termina de inmediato | `stallCurrentAmps` muy bajo: está confundiendo el pico de arranque con el tope. Súbelo, o sube `stallDebounceSeconds` |
| Mantengo X y sigue sin buscar | `Demo/Limelight OK`. Si está en rojo, revisa `turretCameraName` — es el error número uno |
| Barre y nunca engancha | `Demo/Torreta` en `BUSCANDO` permanente → pipelines mal expuestos, o lente desenfocado |
| La torreta oscila | Sección 7. Empieza por `turretExtraLatencySeconds` |
| Tiembla en sitio | Sube `turretSetpointDeadbandRad` |
| Pierde el tag cuando otro se acerca | `useLimelightPriorityId` debe estar en true, y fija el tag con Back |
| El follow-me se aleja | `followTagYawOffsetRad`, luego `followOrbitStrength = 0.0` |
| El hub orbit vibra | Sube `headingToleranceRad` |
| La odometría se vuelve loca | `trustVisionPoseInDemo` debe estar en false fuera de cancha |
| Engancha lejos pero falla cerca | Sobreexposición: el borde blanco se derrama sobre el negro. Baja exposición y gain |
| Engancha y desengancha sin patrón | Parpadeo de las lámparas (sección 5.6). Cambia al pipeline 2 con Start |
| El tag se ve grande pero no lo detecta | Reflejo del techo. Inclina el tag 10-15° hacia adelante |
| La distancia salta y el flywheel persigue | Normal más allá de 4 m. Ya está limitado por `maxTrustedVisionDistanceMeters` y filtrado |
| Alcance mucho menor al esperado | Revisa que el pipeline esté en 960x720 con **decimate 1.0**, no 2.0 |
| Los tiros salen cortos | Mira `Demo/LISTO PARA TIRAR` antes de alimentar |

---

## 13. Qué sirve para la temporada

**El Smooth Drive System.** Un driver experimentado también le da jalones al
stick en una final. Subir `maxSpeedFraction` a 1.0 y `translationMaxAccel` a ~2.5
te da protección de transmisión sin perder desempeño. Dos números.

**El estimador de torreta con compensación de latencia.** Esto es lo más valioso
de todo el proyecto para competencia. Cualquier apuntado por visión sobre un
mecanismo móvil sufre el mismo problema, y la solución es exactamente la misma.

**El PID de rumbo sin doble escalado.** El código de competencia multiplica la
salida del `ProfiledPIDController` por `getMaxAngularSpeedRadPerSec()` (~20 rad/s),
lo que satura al instante. El demo lo hace bien. Migrarlo a
`DriveCommands.joystickDrive` son tres líneas y arregla que "el orbit es muy
violento".

**La agitación** y **la clave de log separada** (`TurretShooter` vs `Shooter`)
aplican tal cual a los dos modos.

---

## 14. Estado del código

- 60 archivos parseados sin errores de sintaxis
- Cero identificadores sin resolver en el código nuevo
- `elastic-layout.json` valida como JSON (30 widgets, 2 pestañas)
- **No se pudo compilar con Gradle**: el entorno no tiene JDK 17 ni acceso a
  Maven. **Corre `./gradlew build` antes de desplegar.**

### Qué cambió en esta iteración

| Área | Cambio |
|---|---|
| Modo solo | Nuevo flag `isSoloDemo`: todo en un control |
| Volcado suave | Reemplaza al punto fijo: sin corrección, hood plano, potencia fija |
| BOMBER glitchy | **Arreglado**: torreta y cañón fijo compartían distancia divergente |
| Shoot while moving | Objetivo virtual conectado a distancia **y** ángulo (torreta y chasis), apagado por defecto |

### Qué cambió en el rollback anterior

| Área | Antes | Ahora |
|---|---|---|
| Apuntado al HUB | Visión + corrección vectorial | **Odometría**, método de competencia |
| Distancia de tiro | Visión 3D (ruidosa, daba 0.45 m) | **Odometría** |
| Alineado de BOMBER | Visión + odometría con HUB azul fijo | **Odometría** con HUB por alianza, PID suave |
| Follow-me | Torreta rastrea + chasis persigue la torreta | **Sólo chasis**, torreta congelada |
| Tiro suave | 12-22 RPS, mapa oculto | 20-34 RPS, mapa por puntos editable |
| PID de torreta | Continuo, temblaba | **Histéresis de enganche** + ganancias sin kI |
| Orientación del chasis | Una sola, B reescribía la pose | **FIELD / DRIVER** con toggle en X |
| Intake + feeder | — | **Sin cambios**, se conservó todo |

Pendientes de verificar en el robot, en orden:

1. **Versión de LimelightOS** — sin 2024+ no hay AprilTags ni pose dinámica.
   Es lo primero porque bloquea todo lo demás.
2. **Pipeline 0 en 960x720 con decimate 1.0** — es de donde sale el alcance
3. Los seis números de calibración geométrica de la cámara (sección 6)
4. El signo de `turretTxSign`
5. `turretExtraLatencySeconds` — el ajuste principal del wobble. Está en 0.045
   asumiendo 22 FPS; si bajan la resolución, bájenlo también
6. `maxTrustedVisionDistanceMeters` — está en 4.0 como estimación conservadora.
   Verifíquenlo caminando con un tag y viendo dónde `Demo/Distancia m` empieza a
   saltar en la pestaña Diagnostico de Elastic
7. La convención de yaw del tag para el follow-me
8. Los mapas de tiro suave — estimados al ~45%, hay que ajustarlos tirando
   contra una pared
