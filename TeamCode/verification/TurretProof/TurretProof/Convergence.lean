import TurretProof.Step
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.Positivity
import Mathlib.Tactic.SplitIfs
import Mathlib.Tactic.FieldSimp
import Mathlib.Tactic.Ring
import Mathlib.Algebra.Order.Archimedean.Basic
import Mathlib.Data.Real.Archimedean

/-!
# Turret convergence on a still target

The loop is `step`: the controller's decisions (`power`, `settledNext`, `settledTargetNext`) from
one set of encoder reads, then the servo lag (`servo`), the friction (`effective`) and the move
(`angle`). Every one of those is generated (`Step.lean`) from a `TurretStep.java` contract that
OpenJML proves against the turret code; `step` composes them in the order `Turret.updateTurret` and
`TurretSim.step` do.
-/

set_option linter.style.openClassical false
set_option linter.style.header false

noncomputable section
open Classical
namespace TurretStep

/-- The plant, the tuning and the still target. `noise` bounds every encoder read's error. -/
structure Params where
  k : ℝ
  sPos : ℝ
  sNeg : ℝ
  lag : ℝ
  stopMin : ℝ
  stopMax : ℝ
  tol : ℝ
  exitFactor : ℝ
  ffMin : ℝ
  kv : ℝ
  p : ℝ
  ksPos : ℝ
  ksNeg : ℝ
  maxPower : ℝ
  travelMin : ℝ
  travelMax : ℝ
  target : ℝ
  noise : ℝ

/-- The loop's memory and the plant's state. -/
structure St where
  theta : ℝ
  servo : ℝ
  settled : Bool
  settledTarget : ℝ

/-- One loop's period and its three encoder-read errors. -/
structure In where
  dt : ℝ
  n1 : ℝ
  n2 : ℝ
  n3 : ℝ

/-- One control loop and one plant step. -/
def step (P : Params) (s : St) (i : In) : St :=
  let u := power s.settled s.settledTarget P.target s.theta i.n1 i.n2 i.n3 P.tol P.exitFactor
    P.ffMin P.kv P.p P.ksPos P.ksNeg P.maxPower P.travelMin P.travelMax
  let y2 := servo s.servo u i.dt P.lag
  { theta := angle s.theta (effective y2 P.sPos P.sNeg) P.k i.dt P.stopMin P.stopMax
    servo := y2
    settled := decide (settledNext s.settled s.settledTarget P.target s.theta i.n1 P.tol
      P.exitFactor)
    settledTarget := settledTargetNext s.settled s.settledTarget P.target s.theta i.n1 P.tol }

/-- The parameter box: what every theorem below assumes of the plant and the tuning. -/
structure Params.Valid (P : Params) : Prop where
  k_pos : 0 < P.k
  sPos_nonneg : 0 ≤ P.sPos
  sNeg_nonneg : 0 ≤ P.sNeg
  lag_nonneg : 0 ≤ P.lag
  noise_nonneg : 0 ≤ P.noise
  noise_lt : 2 * P.noise < P.tol
  tol_le : P.tol ≤ 10
  exit_ge : 1 ≤ P.exitFactor
  exit_band : 2 * P.noise ≤ (P.exitFactor - 1) * P.tol
  band_small : P.exitFactor * P.tol ≤ 20
  p_nonneg : 0 ≤ P.p
  ksPos_nonneg : 0 ≤ P.ksPos
  ksNeg_nonneg : 0 ≤ P.ksNeg
  maxPower_pos : 0 < P.maxPower
  maxPower_le : P.maxPower ≤ 1
  travel : -75 ≤ P.travelMin ∧ P.travelMin ≤ P.target ∧ P.target ≤ P.travelMax ∧ P.travelMax ≤ 75
  stops : -100 ≤ P.stopMin ∧ P.stopMin ≤ P.travelMin ∧ P.travelMax ≤ P.stopMax ∧ P.stopMax ≤ 100

/-- A loop's inputs are in range: a period in (0, dtMax] and reads within the noise bound. -/
structure In.Valid (P : Params) (dtMax : ℝ) (i : In) : Prop where
  dt_pos : 0 < i.dt
  dt_le : i.dt ≤ dtMax
  n1 : |i.n1| ≤ P.noise
  n2 : |i.n2| ≤ P.noise
  n3 : |i.n3| ≤ P.noise

/-! ## The plant: one loop with no command -/

/-- A command of zero reaches the servo as zero, whatever the gate decides. -/
theorem output_zero (theta n2 n3 lo hi : ℝ) : output 0 theta n2 n3 lo hi = 0 := by
  unfold output
  split_ifs <;> rfl

theorem effective_nonneg {y sP sN : ℝ} (hy : 0 ≤ y) (hP : 0 ≤ sP) :
    0 ≤ effective y sP sN ∧ effective y sP sN ≤ y := by
  unfold effective
  split_ifs <;> constructor <;> linarith

theorem effective_nonpos {y sP sN : ℝ} (hy : y ≤ 0) (hP : 0 ≤ sP) (hN : 0 ≤ sN) :
    y ≤ effective y sP sN ∧ effective y sP sN ≤ 0 := by
  unfold effective
  split_ifs <;> constructor <;> linarith

/-- The angle after a forward move lies between the start and the unclamped end. -/
theorem angle_forward {theta eff k dt lo hi : ℝ} (hlo : lo ≤ theta) (hhi : theta ≤ hi)
    (hmv : 0 ≤ k * eff * dt) :
    theta ≤ angle theta eff k dt lo hi ∧ angle theta eff k dt lo hi ≤ theta + k * eff * dt := by
  unfold angle
  constructor
  · exact le_min hhi (le_trans (by linarith) (le_max_left _ _))
  · exact le_trans (min_le_right _ _) (max_le le_rfl (by linarith))

theorem angle_backward {theta eff k dt lo hi : ℝ} (hlo : lo ≤ theta) (hhi : theta ≤ hi)
    (hmv : k * eff * dt ≤ 0) :
    theta + k * eff * dt ≤ angle theta eff k dt lo hi ∧ angle theta eff k dt lo hi ≤ theta := by
  unfold angle
  constructor
  · exact le_min (by linarith) (le_max_left _ _)
  · exact le_trans (min_le_right _ _) (max_le (by linarith) hlo)

/-- The stops hold the turret: the angle after a loop is between them. -/
theorem angle_within {theta eff k dt lo hi : ℝ} (h : lo ≤ hi) :
    lo ≤ angle theta eff k dt lo hi ∧ angle theta eff k dt lo hi ≤ hi := by
  unfold angle
  exact ⟨le_min h (le_max_right _ _), min_le_left _ _⟩

/-- With no command the servo output decays toward zero, and its remaining coast
(`k * lag * servo`) shrinks by at least this loop's move. -/
theorem coast_forward {y k dt lag : ℝ} (hy : 0 ≤ y) (hk : 0 ≤ k) (hdt : 0 < dt)
    (hlag : 0 ≤ lag) :
    0 ≤ servo y 0 dt lag ∧ servo y 0 dt lag ≤ y ∧
      k * dt * servo y 0 dt lag + k * lag * servo y 0 dt lag ≤ k * lag * y := by
  unfold servo
  split_ifs with h
  · refine ⟨le_rfl, hy, ?_⟩
    simp only [mul_zero, add_zero]
    positivity
  · rw [not_le] at h
    have hl : 0 < lag := lt_of_lt_of_le hdt h.le
    have hf : dt / lag < 1 := (div_lt_one hl).2 h
    have hf0 : 0 < dt / lag := div_pos hdt hl
    have e1 : y + (0 - y) * (dt / lag) = y * (1 - dt / lag) := by ring
    rw [e1]
    refine ⟨mul_nonneg hy (by linarith), by nlinarith, ?_⟩
    have key : dt * (1 - dt / lag) + lag * (1 - dt / lag) ≤ lag := by
      field_simp
      nlinarith [sq_nonneg dt]
    nlinarith [mul_le_mul_of_nonneg_left key (mul_nonneg hk hy)]

theorem coast_backward {y k dt lag : ℝ} (hy : y ≤ 0) (hk : 0 ≤ k) (hdt : 0 < dt)
    (hlag : 0 ≤ lag) :
    servo y 0 dt lag ≤ 0 ∧ y ≤ servo y 0 dt lag ∧
      k * lag * y ≤ k * dt * servo y 0 dt lag + k * lag * servo y 0 dt lag := by
  have h := coast_forward (y := -y) (by linarith) hk hdt hlag
  have hs : servo (-y) 0 dt lag = -servo y 0 dt lag := by unfold servo; split_ifs <;> ring
  rw [hs] at h
  obtain ⟨h1, h2, h3⟩ := h
  exact ⟨by linarith, by linarith, by nlinarith⟩

/-! ## Holding: once settled, the turret stays settled and aimed -/

/-- The hold set: settled on (nearly) this target, and both the error and the error after the
servo's whole remaining coast (`k * lag * servo`) within `exitFactor * tol - noise`. -/
def Hold (P : Params) (s : St) : Prop :=
  s.settled = true ∧ |P.target - s.settledTarget| ≤ P.tol ∧ |s.servo| ≤ 1 ∧
    |P.target - s.theta| ≤ P.exitFactor * P.tol - P.noise ∧
    |P.target - s.theta - P.k * P.lag * s.servo| ≤ P.exitFactor * P.tol - P.noise ∧
    P.stopMin ≤ s.theta ∧ s.theta ≤ P.stopMax

/-- In the hold set the controller commands nothing. -/
theorem power_hold {P : Params} {s : St} {i : In} (hP : P.Valid) (hs : Hold P s)
    (hn1 : |i.n1| ≤ P.noise) :
    power s.settled s.settledTarget P.target s.theta i.n1 i.n2 i.n3 P.tol P.exitFactor P.ffMin
      P.kv P.p P.ksPos P.ksNeg P.maxPower P.travelMin P.travelMax = 0 ∧
    settledNext s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor ∧
    settledTargetNext s.settled s.settledTarget P.target s.theta i.n1 P.tol = s.settledTarget := by
  obtain ⟨hset, hband, -, he, -, -⟩ := hs
  have hb : holdsBand s.settled s.settledTarget P.target P.tol := ⟨hset, hband⟩
  have hm : |measuredError P.target s.theta i.n1| ≤ P.exitFactor * P.tol := by
    unfold measuredError
    calc |P.target - (s.theta + i.n1)| = |(P.target - s.theta) - i.n1| := by ring_nf
      _ ≤ |P.target - s.theta| + |i.n1| := abs_sub _ _
      _ ≤ P.exitFactor * P.tol := by linarith
  have hsn : settledNext s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor := by
    unfold settledNext; rw [if_pos hb]; exact hm
  refine ⟨?_, hsn, ?_⟩
  · unfold power; rw [if_pos hsn]; exact output_zero _ _ _ _ _
  · unfold settledTargetNext; rw [if_pos hb]

/-- **Theorem (hold).** From the hold set, one loop with any in-range inputs ends in the hold set:
once settled on a still target, the turret stays settled, and aimed within the exit band. -/
theorem hold_step {P : Params} {s : St} {i : In} {dtMax : ℝ} (hP : P.Valid)
    (hi : i.Valid P dtMax) (hs : Hold P s) : Hold P (step P s i) := by
  obtain ⟨hu, hsn, hst⟩ := power_hold hP hs hi.n1
  have hs' := hs
  obtain ⟨hset, hband, hy, he, hc, hlo, hhi⟩ := hs'
  have hk := hP.k_pos.le
  have hstop : P.stopMin ≤ P.stopMax := by
    obtain ⟨_, h1, h2, _⟩ := hP.stops; obtain ⟨_, h3, h4, _⟩ := hP.travel; linarith
  simp only [step, Hold, hu, decide_eq_true_eq]
  refine ⟨hsn, by rw [hst]; exact hband, ?_⟩
  have hwin := angle_within (theta := s.theta)
    (eff := effective (servo s.servo 0 i.dt P.lag) P.sPos P.sNeg) (k := P.k) (dt := i.dt) hstop
  rcases le_total 0 s.servo with h0 | h0
  · obtain ⟨y0, yle, ycoast⟩ := coast_forward h0 hk hi.dt_pos hP.lag_nonneg
    obtain ⟨f0, fle⟩ := effective_nonneg y0 hP.sPos_nonneg (sN := P.sNeg)
    have hmv0 : 0 ≤ P.k * effective (servo s.servo 0 i.dt P.lag) P.sPos P.sNeg * i.dt :=
      mul_nonneg (mul_nonneg hk f0) hi.dt_pos.le
    have hmv : P.k * effective (servo s.servo 0 i.dt P.lag) P.sPos P.sNeg * i.dt
        ≤ P.k * i.dt * servo s.servo 0 i.dt P.lag := by
      have := mul_le_mul_of_nonneg_left fle (mul_nonneg hk hi.dt_pos.le); nlinarith
    obtain ⟨a1, a2⟩ := angle_forward hlo hhi hmv0
    have hkl : 0 ≤ P.k * P.lag * servo s.servo 0 i.dt P.lag :=
      mul_nonneg (mul_nonneg hk hP.lag_nonneg) y0
    have hkl0 : 0 ≤ P.k * P.lag * s.servo := mul_nonneg (mul_nonneg hk hP.lag_nonneg) h0
    have hy' := abs_le.mp hy
    refine ⟨abs_le.mpr ⟨by linarith, by linarith⟩, ?_, ?_, hwin.1, hwin.2⟩
    · rw [abs_le] at he hc ⊢; constructor <;> linarith
    · rw [abs_le] at he hc ⊢; constructor <;> linarith
  · obtain ⟨y0, yle, ycoast⟩ := coast_backward h0 hk hi.dt_pos hP.lag_nonneg
    obtain ⟨f0, fle⟩ := effective_nonpos y0 hP.sPos_nonneg hP.sNeg_nonneg
    have hmv0 : P.k * effective (servo s.servo 0 i.dt P.lag) P.sPos P.sNeg * i.dt ≤ 0 :=
      mul_nonpos_of_nonpos_of_nonneg (mul_nonpos_of_nonneg_of_nonpos hk fle) hi.dt_pos.le
    have hmv : P.k * i.dt * servo s.servo 0 i.dt P.lag
        ≤ P.k * effective (servo s.servo 0 i.dt P.lag) P.sPos P.sNeg * i.dt := by
      have := mul_le_mul_of_nonneg_left f0 (mul_nonneg hk hi.dt_pos.le); nlinarith
    obtain ⟨a1, a2⟩ := angle_backward hlo hhi hmv0
    have hkl : P.k * P.lag * servo s.servo 0 i.dt P.lag ≤ 0 :=
      mul_nonpos_of_nonneg_of_nonpos (mul_nonneg hk hP.lag_nonneg) y0
    have hkl0 : P.k * P.lag * s.servo ≤ 0 :=
      mul_nonpos_of_nonneg_of_nonpos (mul_nonneg hk hP.lag_nonneg) h0
    have hy' := abs_le.mp hy
    refine ⟨abs_le.mpr ⟨by linarith, by linarith⟩, ?_, ?_, hwin.1, hwin.2⟩
    · rw [abs_le] at he hc ⊢; constructor <;> linarith
    · rw [abs_le] at he hc ⊢; constructor <;> linarith

/-! ## Approaching from below: the target above the turret -/

/-- The conditions the gains must meet, for a plant in the box and loop periods up to `dtMax`.
`m` is the drive margin over the plant's friction at the band's edge; `a`, `b` bound the servo
output by the error (`servo <= a * error + b`); `reach` keeps one loop from carrying the turret past
the band's far edge; `land` keeps the coast after settling inside the exit band. -/
structure Witness (P : Params) (dtMax a b m : ℝ) : Prop where
  a_nonneg : 0 ≤ a
  b_nonneg : 0 ≤ b
  m_pos : 0 < m
  m_le_max : m ≤ P.maxPower - P.sPos
  m_le_drive : m ≤ P.p * P.tol + P.ksPos - P.sPos
  akl : a * P.k * P.lag < 1
  dtMax_pos : 0 < dtMax
  slope : (1 + a * P.k * dtMax) * P.p ≤ (1 - a * P.k * P.lag) * a
  edge : (1 + a * P.k * dtMax) * (P.p * P.tol + P.ksPos)
    ≤ (1 - a * P.k * P.lag) * (a * (P.tol - P.noise) + b)
  reach : P.k * dtMax * b ≤ (P.tol - P.noise) * (2 + a * P.k * dtMax)
  land : (P.tol - P.noise) * (1 - P.k * P.lag * a) + P.k * P.lag * b ≤ P.exitFactor * P.tol - P.noise
  clampOk : P.k * dtMax * (m / 2) ≤ P.tol - P.noise

/-- The approach set from below: not settled on this target, short of the band's far edge, inside
the travel, and the servo output forward and at most `a * error + b`. -/
def ApproachUp (P : Params) (a b : ℝ) (s : St) : Prop :=
  ¬holdsBand s.settled s.settledTarget P.target P.tol ∧ -(P.tol - P.noise) < P.target - s.theta ∧
    P.travelMin ≤ s.theta ∧ s.theta ≤ P.stopMax ∧ 0 ≤ s.servo ∧ s.servo ≤ 1 ∧
    s.servo ≤ a * (P.target - s.theta) + b

/-- The ranking measure while approaching from below; every driving loop lowers it. -/
def potentialUp (P : Params) (m : ℝ) (s : St) : ℝ :=
  (P.target - s.theta) + (P.tol - P.noise) + P.k * P.lag * max 0 (P.sPos + m / 2 - s.servo)

theorem potentialUp_nonneg {P : Params} {a b m : ℝ} {s : St} (hP : P.Valid)
    (hs : ApproachUp P a b s) : 0 ≤ potentialUp P m s := by
  have := mul_nonneg (mul_nonneg hP.k_pos.le hP.lag_nonneg) (le_max_left 0 (P.sPos + m / 2 - s.servo))
  unfold potentialUp; linarith [hs.2.1]

/-- Driving up, the controller gives the P + kS command, limited: at least the margin over the
plant's friction, at most what the true error asks for. -/
theorem power_driveUp {P : Params} {a b m dtMax : ℝ} {s : St} {i : In} (hP : P.Valid)
    (hW : Witness P dtMax a b m) (hi : i.Valid P dtMax) (hs : ApproachUp P a b s)
    (hd : P.target - (s.theta + i.n1) > P.tol) :
    let u := power s.settled s.settledTarget P.target s.theta i.n1 i.n2 i.n3 P.tol P.exitFactor
      P.ffMin P.kv P.p P.ksPos P.ksNeg P.maxPower P.travelMin P.travelMax
    P.sPos + m ≤ u ∧ u ≤ P.p * (P.target - s.theta + P.noise) + P.ksPos ∧ 0 < u ∧ u ≤ 1 ∧
      ¬settledNext s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor := by
  intro u
  obtain ⟨hnb, -, hmin, -, -⟩ := hs
  obtain ⟨hn1a, hn1b⟩ := abs_le.mp hi.n1
  obtain ⟨hn2a, hn2b⟩ := abs_le.mp hi.n2
  obtain ⟨hn3a, hn3b⟩ := abs_le.mp hi.n3
  obtain ⟨htr1, htr2, htr3, htr4⟩ := hP.travel
  have hnz := hP.noise_lt; have hn0 := hP.noise_nonneg; have htl := hP.tol_le
  have hsn : ¬settledNext s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor := by
    unfold settledNext measuredError; rw [if_neg hnb, abs_of_pos (by linarith)]; linarith
  have hem : P.p * P.tol ≤ P.p * (P.target - (s.theta + i.n1)) :=
    mul_le_mul_of_nonneg_left hd.le hP.p_nonneg
  have hem2 : P.p * (P.target - (s.theta + i.n1)) ≤ P.p * (P.target - s.theta + P.noise) :=
    mul_le_mul_of_nonneg_left (by linarith) hP.p_nonneg
  have hc : driveCommand (measuredError P.target s.theta i.n1) P.p P.ksPos P.ksNeg P.maxPower
      = min P.maxPower (P.p * (P.target - (s.theta + i.n1)) + P.ksPos) := by
    unfold driveCommand measuredError; rw [if_pos (by linarith)]
  set c := min P.maxPower (P.p * (P.target - (s.theta + i.n1)) + P.ksPos) with hcdef
  have hcm : P.sPos + m ≤ c := le_min (by linarith [hW.m_le_max]) (by linarith [hW.m_le_drive])
  have hcpos : 0 < c := by linarith [hP.sPos_nonneg, hW.m_pos]
  have hc1 : c ≤ 1 := le_trans (min_le_left _ _) hP.maxPower_le
  have hnp2 : ¬pushesPast c (s.theta + i.n2) P.travelMin P.travelMax := by
    unfold pushesPast; push Not; constructor <;> intro <;> linarith
  have hnp3 : ¬pushesPast c (s.theta + i.n3) P.travelMin P.travelMax := by
    unfold pushesPast; push Not; constructor <;> intro <;> linarith
  have hu : u = c := by
    show power _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ = c
    unfold power; rw [if_neg hsn, hc]; unfold output
    rw [if_neg hnp2, if_pos ⟨⟨by linarith, by linarith⟩, hnp3⟩]
  rw [hu]
  exact ⟨hcm, le_trans (min_le_right _ _) (by linarith), hcpos, hc1, hsn⟩

/-- The drive is under the servo bound at every error beyond the band and every loop period up to
`dtMax`: `(1 + a k dt) (p (e + noise) + kS) <= (1 - a k lag) (a e + b)`. -/
theorem drive_bound {P : Params} {a b m dtMax dt e : ℝ} (hP : P.Valid) (hW : Witness P dtMax a b m)
    (hdt : 0 < dt) (hdtm : dt ≤ dtMax) (he : P.tol - P.noise ≤ e) :
    (1 + a * P.k * dt) * (P.p * (e + P.noise) + P.ksPos)
      ≤ (1 - a * P.k * P.lag) * (a * e + b) := by
  have hak : 0 ≤ a * P.k := mul_nonneg hW.a_nonneg hP.k_pos.le
  have hU : 0 ≤ P.p * (e + P.noise) + P.ksPos := by
    have := hP.noise_lt; have := hP.noise_nonneg
    have : 0 ≤ e + P.noise := by linarith
    have := mul_nonneg hP.p_nonneg this; linarith [hP.ksPos_nonneg]
  have hx : 0 ≤ e - (P.tol - P.noise) := by linarith
  have h1 : (1 + a * P.k * dt) * (P.p * (e + P.noise) + P.ksPos)
      ≤ (1 + a * P.k * dtMax) * (P.p * (e + P.noise) + P.ksPos) :=
    mul_le_mul_of_nonneg_right (by nlinarith) hU
  have h2 : (1 + a * P.k * dtMax) * P.p * (e - (P.tol - P.noise))
      ≤ (1 - a * P.k * P.lag) * a * (e - (P.tol - P.noise)) :=
    mul_le_mul_of_nonneg_right hW.slope hx
  nlinarith [hW.edge]

/-- One loop of servo lag driven toward `0 < u <= U` keeps the output under its bound: with `0 <= y
<= Y` and `(1 + q dt) U <= (1 - q lag) Y`, the new output `y2` has `0 <= y2` and `(1 + q dt) y2 <=
Y`. -/
theorem servo_up {y u Y U q dt lag : ℝ} (hy : 0 ≤ y) (hyY : y ≤ Y) (hu : 0 < u) (huU : u ≤ U)
    (hq : 0 ≤ q) (hdt : 0 < dt) (hlag : 0 ≤ lag) (hql : q * lag < 1)
    (hc : (1 + q * dt) * U ≤ (1 - q * lag) * Y) :
    0 ≤ servo y u dt lag ∧ servo y u dt lag * (1 + q * dt) ≤ Y := by
  have hY : 0 ≤ Y := le_trans hy hyY
  unfold servo
  split_ifs with h
  · refine ⟨hu.le, ?_⟩
    have : 0 ≤ q * lag * Y := mul_nonneg (mul_nonneg hq hlag) hY
    nlinarith [mul_le_mul_of_nonneg_left huU (by positivity : (0:ℝ) ≤ 1 + q * dt)]
  · rw [not_le] at h
    have hl : 0 < lag := lt_of_lt_of_le hdt h.le
    set al := dt / lag with hal
    have hal0 : 0 < al := div_pos hdt hl
    have hal1 : al < 1 := (div_lt_one hl).2 h
    have hdtl : dt = al * lag := by rw [hal]; field_simp
    have hc1 : 0 ≤ 1 + q * dt := by positivity
    refine ⟨by nlinarith, ?_⟩
    have e1 : (y + (u - y) * al) * (1 + q * dt)
        = (1 - al) * ((1 + q * dt) * y) + al * ((1 + q * dt) * u) := by ring
    rw [e1]
    have t1 : (1 - al) * ((1 + q * dt) * y) ≤ (1 - al) * ((1 + q * dt) * Y) :=
      mul_le_mul_of_nonneg_left (mul_le_mul_of_nonneg_left hyY hc1) (by linarith)
    have t2 : al * ((1 + q * dt) * u) ≤ al * ((1 - q * lag) * Y) :=
      mul_le_mul_of_nonneg_left (le_trans (mul_le_mul_of_nonneg_left huU hc1) hc) hal0.le
    have t3 : (1 - al) * ((1 + q * dt) * Y) + al * ((1 - q * lag) * Y) = Y - q * al * dt * Y := by
      rw [hdtl]; ring
    have t4 : 0 ≤ q * al * dt * Y := by positivity
    linarith

/-- `reach` at `dtMax` holds at every shorter loop period. -/
theorem reach_dt {P : Params} {a b m dtMax dt : ℝ} (hP : P.Valid) (hW : Witness P dtMax a b m)
    (hdt : 0 < dt) (hdtm : dt ≤ dtMax) :
    P.k * dt * b ≤ (P.tol - P.noise) * (2 + a * P.k * dt) := by
  have hak : 0 ≤ a * P.k := mul_nonneg hW.a_nonneg hP.k_pos.le
  have ht : 0 < P.tol - P.noise := by linarith [hP.noise_lt, hP.noise_nonneg]
  -- k dt b - (tol - noise)(2 + a k dt) is linear in dt, negative at 0 and nonpositive at dtMax.
  have hlin : P.k * dt * b - (P.tol - P.noise) * (2 + a * P.k * dt)
      = (dt / dtMax) * (P.k * dtMax * b - (P.tol - P.noise) * (2 + a * P.k * dtMax))
        - (1 - dt / dtMax) * (2 * (P.tol - P.noise)) := by
    have hne : dtMax ≠ 0 := ne_of_gt hW.dtMax_pos
    field_simp; ring
  have hf0 : 0 ≤ dt / dtMax := div_nonneg hdt.le hW.dtMax_pos.le
  have hf1 : dt / dtMax ≤ 1 := (div_le_one hW.dtMax_pos).2 hdtm
  have := mul_nonpos_of_nonneg_of_nonpos hf0 (by linarith [hW.reach] :
    P.k * dtMax * b - (P.tol - P.noise) * (2 + a * P.k * dtMax) ≤ 0)
  have := mul_nonneg (by linarith : 0 ≤ 1 - dt / dtMax) (by linarith : 0 ≤ 2 * (P.tol - P.noise))
  linarith

theorem lag_fraction (k dt : ℝ) {lag : ℝ} (hl : 0 < lag) : k * lag * (dt / lag) = k * dt := by
  field_simp

theorem servo_le_one {y u dt lag : ℝ} (hy : y ≤ 1) (hu : u ≤ 1) (hdt : 0 < dt) (hlag : 0 ≤ lag) :
    servo y u dt lag ≤ 1 := by
  unfold servo
  split_ifs with h
  · exact hu
  · rw [not_le] at h
    have hl : 0 < lag := lt_of_lt_of_le hdt h.le
    have h0 : 0 < dt / lag := div_pos hdt hl
    have h1 : dt / lag < 1 := (div_lt_one hl).2 h
    nlinarith

theorem angle_exact {theta eff k dt lo hi : ℝ} (hlo : lo ≤ theta + k * eff * dt)
    (hhi : theta + k * eff * dt ≤ hi) : angle theta eff k dt lo hi = theta + k * eff * dt := by
  unfold angle; rw [max_eq_left hlo, min_eq_right hhi]

set_option maxHeartbeats 1000000 in
/-- **Theorem (approach makes progress).** From the approach set, a loop that measures the error
beyond the band drives the turret, stays in the approach set, and lowers the ranking measure by
`k dt m / 2`. -/
theorem approach_up_drive {P : Params} {a b m dtMax : ℝ} {s : St} {i : In} (hP : P.Valid)
    (hW : Witness P dtMax a b m) (hi : i.Valid P dtMax) (hs : ApproachUp P a b s)
    (hd : P.target - (s.theta + i.n1) > P.tol) :
    ApproachUp P a b (step P s i) ∧
      potentialUp P m (step P s i) ≤ potentialUp P m s - P.k * i.dt * (m / 2) := by
  obtain ⟨hu_lo, hu_hi, hu0, hu1, hsn⟩ := power_driveUp hP hW hi hs hd
  have hsn' : settledNext s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor
      = False := propext ⟨fun h => hsn h, False.elim⟩
  simp only [step, ApproachUp, potentialUp, holdsBand, hsn', decide_false]
  have hs' := hs
  obtain ⟨-, -, hmin, hmax, hy0, hy1, hyb⟩ := hs'
  set u := power s.settled s.settledTarget P.target s.theta i.n1 i.n2 i.n3 P.tol P.exitFactor
    P.ffMin P.kv P.p P.ksPos P.ksNeg P.maxPower P.travelMin P.travelMax with hudef
  set e := P.target - s.theta with hedef
  obtain ⟨hn1a, hn1b⟩ := abs_le.mp hi.n1
  obtain ⟨htr1, htr2, htr3, htr4⟩ := hP.travel
  have hk := hP.k_pos
  have hdt := hi.dt_pos
  have hak : 0 ≤ a * P.k := mul_nonneg hW.a_nonneg hk.le
  have he : P.tol - P.noise ≤ e := by linarith
  -- The servo output stays under its bound.
  have hdb := drive_bound hP hW hdt hi.dt_le he
  obtain ⟨hy2, hy2b⟩ := servo_up hy0 hyb hu0 hu_hi hak hdt hP.lag_nonneg hW.akl hdb
  set y2 := servo s.servo u i.dt P.lag with hy2def
  have hy21 : y2 ≤ 1 := servo_le_one hy1 hu1 hdt hP.lag_nonneg
  obtain ⟨hf0, hfle⟩ := effective_nonneg hy2 hP.sPos_nonneg (sN := P.sNeg)
  set eff := effective y2 P.sPos P.sNeg with heffdef
  have hmv0 : 0 ≤ P.k * eff * i.dt := mul_nonneg (mul_nonneg hk.le hf0) hdt.le
  have hmvle : P.k * eff * i.dt ≤ P.k * i.dt * y2 := by
    have := mul_le_mul_of_nonneg_left hfle (mul_nonneg hk.le hdt.le); nlinarith
  -- One loop does not carry the turret past the band's far edge.
  have hreach := reach_dt hP hW hdt hi.dt_le
  have hc0 : 0 < 1 + a * P.k * i.dt := by positivity
  have hover : e - P.k * i.dt * y2 > -(P.tol - P.noise) := by
    have h1 : (e - P.k * i.dt * y2) * (1 + a * P.k * i.dt) ≥ e - P.k * i.dt * b := by
      have := mul_le_mul_of_nonneg_left hy2b (mul_nonneg hk.le hdt.le); nlinarith
    have ht : 0 < P.tol - P.noise := by linarith [hP.noise_lt, hP.noise_nonneg]
    by_contra hc
    push Not at hc
    have := mul_le_mul_of_nonneg_right hc hc0.le
    nlinarith
  -- The stops: the turret ends between them, and no further than the unclamped move.
  obtain ⟨hst1, hst2, hst3, hst4⟩ := hP.stops
  have hstop : P.stopMin ≤ P.stopMax := by linarith
  obtain ⟨a1, a2⟩ := angle_forward (lo := P.stopMin) (hi := P.stopMax) (by linarith) hmax hmv0
  have hwin := angle_within (theta := s.theta) (eff := eff) (k := P.k) (dt := i.dt) hstop
  have hnoclamp : s.theta + P.k * eff * i.dt ≤ P.stopMax →
      angle s.theta eff P.k i.dt P.stopMin P.stopMax = s.theta + P.k * eff * i.dt :=
    fun h => angle_exact (by linarith) h
  have hclamp : P.stopMax < s.theta + P.k * eff * i.dt →
      angle s.theta eff P.k i.dt P.stopMin P.stopMax = P.stopMax := fun h => by
    unfold angle; rw [max_eq_left (by linarith), min_eq_left h.le]
  obtain ⟨th, hth⟩ : ∃ x, x = angle s.theta eff P.k i.dt P.stopMin P.stopMax := ⟨_, rfl⟩
  rw [← hth] at a1 a2 hwin hnoclamp hclamp ⊢
  clear_value eff y2 u e
  clear hsn' hsn
  -- Still approaching: short of the band's far edge, and the servo under its bound.
  have hab : a * (P.k * eff * i.dt) ≤ a * (P.k * i.dt * y2) :=
    mul_le_mul_of_nonneg_left hmvle hW.a_nonneg
  have hab2 : a * (e - P.k * eff * i.dt) ≤ a * (P.target - th) :=
    mul_le_mul_of_nonneg_left (by linarith only [a2, hedef]) hW.a_nonneg
  have id1 : a * (e - P.k * eff * i.dt) = a * e - a * (P.k * eff * i.dt) := by ring
  have id2 : y2 * (1 + a * P.k * i.dt) = y2 + a * (P.k * i.dt * y2) := by ring
  refine ⟨⟨by simp, by linarith only [a2, hmvle, hover, hedef], by linarith only [a1, hmin],
    hwin.2, hy2, hy21, by linarith only [hy2b, hab, hab2, id1, id2]⟩, ?_⟩
  -- The ranking measure drops by k dt m / 2.
  have hkl : 0 ≤ P.k * P.lag := mul_nonneg hk.le hP.lag_nonneg
  have hg := le_max_left 0 (P.sPos + m / 2 - s.servo)
  have hkg : 0 ≤ P.k * P.lag * max 0 (P.sPos + m / 2 - s.servo) := mul_nonneg hkl hg
  rcases le_or_gt (P.sPos + m / 2) y2 with hge | hlt
  · -- Past the friction: the turret moves at least k dt m / 2, or a stop past the target holds it.
    have heff : eff = y2 - P.sPos := by
      rw [heffdef]; unfold effective
      rw [if_pos hy2, if_neg (by linarith [hW.m_pos])]
    have hprog : P.k * i.dt * (m / 2) ≤ P.k * eff * i.dt := by
      rw [heff]; have := mul_le_mul_of_nonneg_left (by linarith : m / 2 ≤ y2 - P.sPos)
        (mul_nonneg hk.le hdt.le); nlinarith
    have hdtm : P.k * i.dt * (m / 2) ≤ P.k * dtMax * (m / 2) :=
      mul_le_mul_of_nonneg_right (mul_le_mul_of_nonneg_left hi.dt_le hk.le)
        (by linarith only [hW.m_pos])
    have hclampOk := hW.clampOk
    have hnew : P.target - th ≤ e - P.k * i.dt * (m / 2) := by
      rcases le_or_gt (s.theta + P.k * eff * i.dt) P.stopMax with hc | hc
      · rw [hnoclamp hc]; linarith only [hprog, hedef]
      · rw [hclamp hc]; linarith only [htr3, hst3, he, hclampOk, hdtm]
    have hmax0 : max 0 (P.sPos + m / 2 - y2) = 0 := max_eq_left (sub_nonpos.mpr hge)
    rw [hmax0, mul_zero]
    linarith only [hnew, hkg]
  · -- Short of the friction: the servo output closes on it by dt / lag of the way.
    have hlag : i.dt < P.lag := by
      by_contra hc; push Not at hc
      have : y2 = u := by rw [hy2def]; unfold servo; rw [if_pos hc]
      linarith [hW.m_pos]
    have hl : 0 < P.lag := lt_trans hdt hlag
    obtain ⟨al, hal⟩ : ∃ x : ℝ, x = i.dt / P.lag := ⟨_, rfl⟩
    have hal0 : 0 < al := by rw [hal]; exact div_pos hdt hl
    have hal1 : al < 1 := by rw [hal]; exact (div_lt_one hl).2 hlag
    have hy2e : y2 = s.servo + (u - s.servo) * al := by
      rw [hy2def]; unfold servo; rw [if_neg (not_le.mpr hlag), ← hal]
    have hid : y2 - (P.sPos + m / 2)
        = (s.servo - (P.sPos + m / 2)) * (1 - al) + (u - (P.sPos + m / 2)) * al := by
      rw [hy2e]; ring
    have hm := hW.m_pos
    have hyg : s.servo < P.sPos + m / 2 := by
      by_contra hc; push Not at hc
      have h1 := mul_nonneg (sub_nonneg.2 hc) (by linarith only [hal1] : (0:ℝ) ≤ 1 - al)
      have h2 := mul_pos (by linarith only [hu_lo, hm] : (0:ℝ) < u - (P.sPos + m / 2)) hal0
      linarith only [hid, h1, h2, hlt]
    have hx1 : (0:ℝ) ≤ P.sPos + m / 2 - y2 := sub_nonneg.mpr hlt.le
    have hx2 : (0:ℝ) ≤ P.sPos + m / 2 - s.servo := sub_nonneg.mpr hyg.le
    have hmax1 : max 0 (P.sPos + m / 2 - y2) = P.sPos + m / 2 - y2 := max_eq_right hx1
    have hmax2 : max 0 (P.sPos + m / 2 - s.servo) = P.sPos + m / 2 - s.servo := max_eq_right hx2
    rw [hmax1, hmax2]
    have hkal : P.k * P.lag * al = P.k * i.dt := by rw [hal]; exact lag_fraction P.k i.dt hl
    have hgap : P.sPos + m / 2 - y2
        = (P.sPos + m / 2 - s.servo) - al * (P.sPos + m / 2 - s.servo)
          - al * (u - (P.sPos + m / 2)) := by rw [hy2e]; ring
    have h1 : 0 ≤ P.k * P.lag * (al * (P.sPos + m / 2 - s.servo)) :=
      mul_nonneg hkl (mul_nonneg hal0.le (by linarith only [hyg]))
    have h2 : P.k * P.lag * (al * (m / 2)) ≤ P.k * P.lag * (al * (u - (P.sPos + m / 2))) :=
      mul_le_mul_of_nonneg_left (mul_le_mul_of_nonneg_left (by linarith only [hu_lo]) hal0.le) hkl
    have h3 : P.k * P.lag * (al * (m / 2)) = P.k * i.dt * (m / 2) := by
      rw [← hkal]; ring
    have key : P.k * P.lag * (P.sPos + m / 2 - y2)
        ≤ P.k * P.lag * (P.sPos + m / 2 - s.servo) - P.k * i.dt * (m / 2) := by
      rw [hgap]
      have e : P.k * P.lag * ((P.sPos + m / 2 - s.servo) - al * (P.sPos + m / 2 - s.servo)
          - al * (u - (P.sPos + m / 2)))
          = P.k * P.lag * (P.sPos + m / 2 - s.servo) - P.k * P.lag * (al * (P.sPos + m / 2 - s.servo))
            - P.k * P.lag * (al * (u - (P.sPos + m / 2))) := by ring
      rw [e]; linarith only [h1, h2, h3]
    linarith only [key, a1, hedef]

/-- **Theorem (approach settles into hold).** From the approach set, a loop that measures the error
within the band settles, and the coast that follows stays inside the exit band: it ends in the hold
set. -/
theorem approach_up_settle {P : Params} {a b m dtMax : ℝ} {s : St} {i : In} (hP : P.Valid)
    (hW : Witness P dtMax a b m) (hi : i.Valid P dtMax) (hs : ApproachUp P a b s)
    (hd : |P.target - (s.theta + i.n1)| ≤ P.tol) : Hold P (step P s i) := by
  obtain ⟨hnb, hfar, hmin, hmax, hy0, hy1, hyb⟩ := hs
  obtain ⟨hn1a, hn1b⟩ := abs_le.mp hi.n1
  obtain ⟨hda, hdb⟩ := abs_le.mp hd
  obtain ⟨htr1, htr2, htr3, htr4⟩ := hP.travel
  have hstops := hP.stops
  have hk := hP.k_pos
  have hsn : settledNext s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor := by
    unfold settledNext measuredError; rw [if_neg hnb]; exact hd
  have hst : settledTargetNext s.settled s.settledTarget P.target s.theta i.n1 P.tol = P.target := by
    unfold settledTargetNext measuredError; rw [if_neg hnb, if_pos hd]
  have hu : power s.settled s.settledTarget P.target s.theta i.n1 i.n2 i.n3 P.tol P.exitFactor
      P.ffMin P.kv P.p P.ksPos P.ksNeg P.maxPower P.travelMin P.travelMax = 0 := by
    unfold power; rw [if_pos hsn]; exact output_zero _ _ _ _ _
  simp only [step, Hold, hu, hst, hsn, decide_true, sub_self, abs_zero]
  obtain ⟨y0, yle, ycoast⟩ := coast_forward hy0 hk.le hi.dt_pos hP.lag_nonneg
  obtain ⟨y2, hy2⟩ : ∃ x, x = servo s.servo 0 i.dt P.lag := ⟨_, rfl⟩
  rw [← hy2] at y0 yle ycoast ⊢
  obtain ⟨f0, fle⟩ := effective_nonneg y0 hP.sPos_nonneg (sN := P.sNeg)
  obtain ⟨eff, heff⟩ : ∃ x, x = effective y2 P.sPos P.sNeg := ⟨_, rfl⟩
  rw [← heff] at f0 fle ⊢
  have hmv0 : 0 ≤ P.k * eff * i.dt := mul_nonneg (mul_nonneg hk.le f0) hi.dt_pos.le
  have hmv : P.k * eff * i.dt ≤ P.k * i.dt * y2 := by
    have := mul_le_mul_of_nonneg_left fle (mul_nonneg hk.le hi.dt_pos.le); nlinarith
  obtain ⟨hst1, hst2, hst3, hst4⟩ := hstops
  have hlo : P.stopMin ≤ s.theta := by linarith
  obtain ⟨a1, a2⟩ := angle_forward hlo hmax hmv0
  have hwin := angle_within (theta := s.theta) (eff := eff) (k := P.k) (dt := i.dt)
    (by linarith : P.stopMin ≤ P.stopMax)
  obtain ⟨th, hth⟩ : ∃ x, x = angle s.theta eff P.k i.dt P.stopMin P.stopMax := ⟨_, rfl⟩
  rw [← hth] at a1 a2 hwin ⊢
  -- The coast from where the band was met stays inside the exit band (`land`).
  have hkl : 0 ≤ P.k * P.lag := mul_nonneg hk.le hP.lag_nonneg
  have hkly : P.k * P.lag * s.servo ≤ P.k * P.lag * (a * (P.target - s.theta) + b) :=
    mul_le_mul_of_nonneg_left hyb hkl
  have hland := hW.land
  have hakl : P.k * P.lag * a < 1 := by have := hW.akl; nlinarith
  have hfar' : -(P.tol - P.noise) * (1 - P.k * P.lag * a)
      ≤ (P.target - s.theta) * (1 - P.k * P.lag * a) :=
    mul_le_mul_of_nonneg_right hfar.le (by linarith)
  have hkly2 : 0 ≤ P.k * P.lag * y2 := mul_nonneg hkl y0
  have hband := hP.exit_band
  have hy21 : |y2| ≤ 1 := abs_le.mpr ⟨by linarith, by linarith⟩
  have hn0 := hP.noise_nonneg
  have id1 : P.k * P.lag * (a * (P.target - s.theta) + b)
      = P.k * P.lag * a * (P.target - s.theta) + P.k * P.lag * b := by ring
  have id2 : (P.target - s.theta) * (1 - P.k * P.lag * a)
      = (P.target - s.theta) - P.k * P.lag * a * (P.target - s.theta) := by ring
  have id3 : -(P.tol - P.noise) * (1 - P.k * P.lag * a)
      = -(P.tol - P.noise) + (P.tol - P.noise) * P.k * P.lag * a := by ring
  have hland' : (P.tol - P.noise) * (1 - P.k * P.lag * a) + P.k * P.lag * b
      = (P.tol - P.noise) - (P.tol - P.noise) * P.k * P.lag * a + P.k * P.lag * b := by ring
  have hlow : -(P.exitFactor * P.tol - P.noise) ≤ P.target - th - P.k * P.lag * y2 := by
    linarith only [a2, hmv, ycoast, hkly, id1, id2, id3, hfar', hland, hland']
  have idb : (P.exitFactor - 1) * P.tol = P.exitFactor * P.tol - P.tol := by ring
  have hup : P.target - th ≤ P.exitFactor * P.tol - P.noise := by
    linarith only [a1, hdb, hn1b, hband, idb]
  refine ⟨trivial, by linarith [hP.noise_lt, hP.noise_nonneg], hy21, ?_, ?_, hwin.1, hwin.2⟩
  · rw [abs_le]; constructor
    · linarith only [hlow, hkly2]
    · exact hup
  · rw [abs_le]; constructor
    · exact hlow
    · linarith only [hup, hkly2]

/-! ## The mirror image: approaching from above -/

/-- The mirrored loop: angles, the target and the travel negated, the friction swapped. -/
def Params.refl (P : Params) : Params :=
  { P with sPos := P.sNeg, sNeg := P.sPos, ksPos := P.ksNeg, ksNeg := P.ksPos,
           travelMin := -P.travelMax, travelMax := -P.travelMin, target := -P.target,
           stopMin := -P.stopMax, stopMax := -P.stopMin }

def St.refl (s : St) : St :=
  { theta := -s.theta, servo := -s.servo, settled := s.settled, settledTarget := -s.settledTarget }

def In.refl (i : In) : In := { dt := i.dt, n1 := -i.n1, n2 := -i.n2, n3 := -i.n3 }

theorem holdsBand_refl (st : Bool) (sT T tol : ℝ) :
    holdsBand st (-sT) (-T) tol ↔ holdsBand st sT T tol := by
  unfold holdsBand; rw [show -T - -sT = -(T - sT) by ring, abs_neg]

theorem settledNext_refl (st : Bool) (sT T th n1 tol X : ℝ) :
    settledNext st (-sT) (-T) (-th) (-n1) tol X ↔ settledNext st sT T th n1 tol X := by
  unfold settledNext measuredError
  rw [show -T - (-th + -n1) = -(T - (th + n1)) by ring, abs_neg]
  by_cases h : holdsBand st sT T tol
  · rw [if_pos ((holdsBand_refl st sT T tol).2 h), if_pos h]
  · rw [if_neg (fun h' => h ((holdsBand_refl st sT T tol).1 h')), if_neg h]

theorem settledTargetNext_refl (st : Bool) (sT T th n1 tol : ℝ) :
    settledTargetNext st (-sT) (-T) (-th) (-n1) tol = -settledTargetNext st sT T th n1 tol := by
  unfold settledTargetNext measuredError
  rw [show -T - (-th + -n1) = -(T - (th + n1)) by ring, abs_neg]
  by_cases h : holdsBand st sT T tol
  · rw [if_pos ((holdsBand_refl st sT T tol).2 h), if_pos h]
  · rw [if_neg (fun h' => h ((holdsBand_refl st sT T tol).1 h')), if_neg h]
    split_ifs <;> ring

theorem driveCommand_refl (e p kP kN M : ℝ) :
    driveCommand (-e) p kN kP M = -driveCommand e p kP kN M := by
  unfold driveCommand
  rcases lt_trichotomy e 0 with h | h | h
  · rw [if_pos (show -e > 0 by linarith), if_neg (show ¬ e > 0 by linarith), if_pos h]
    rw [show p * -e + kN = -(p * e - kN) by ring, ← min_neg_neg, neg_neg]
  · subst h; simp
  · rw [if_neg (show ¬ -e > 0 by linarith), if_pos (show -e < 0 by linarith), if_pos h]
    rw [show p * -e - kP = -(p * e + kP) by ring, ← max_neg_neg]

theorem pushesPast_refl (c a lo hi : ℝ) :
    pushesPast (-c) (-a) (-hi) (-lo) ↔ pushesPast c a lo hi := by
  unfold pushesPast
  constructor
  · rintro (⟨h1, h2⟩ | ⟨h1, h2⟩)
    · right; constructor <;> linarith
    · left; constructor <;> linarith
  · rintro (⟨h1, h2⟩ | ⟨h1, h2⟩)
    · right; constructor <;> linarith
    · left; constructor <;> linarith

theorem output_refl (c th n2 n3 lo hi : ℝ) :
    output (-c) (-th) (-n2) (-n3) (-hi) (-lo) = -output c th n2 n3 lo hi := by
  unfold output
  have e2 : -th + -n2 = -(th + n2) := by ring
  have e3 : -th + -n3 = -(th + n3) := by ring
  rw [e2, e3]
  have hp := pushesPast_refl c (th + n2) lo hi
  by_cases h2 : pushesPast c (th + n2) lo hi
  · rw [if_pos (hp.2 h2), if_pos h2]
    have hz := pushesPast_refl 0 (th + n3) lo hi
    rw [neg_zero] at hz
    by_cases h3 : pushesPast 0 (th + n3) lo hi
    · rw [if_neg (fun h => h.2 (hz.2 h3)), if_neg (fun h => h.2 h3)]; ring
    · split_ifs <;> ring
  · rw [if_neg (fun h => h2 (hp.1 h)), if_neg h2]
    have hq := pushesPast_refl c (th + n3) lo hi
    by_cases h3 : pushesPast c (th + n3) lo hi
    · rw [if_neg (fun h => h.2 (hq.2 h3)), if_neg (fun h => h.2 h3)]; ring
    · by_cases ht : th + n3 ≥ lo - 10 ∧ th + n3 ≤ hi + 10
      · rw [if_pos ⟨⟨by linarith [ht.2], by linarith [ht.1]⟩, fun h => h3 (hq.1 h)⟩,
          if_pos ⟨ht, h3⟩]
      · rw [if_neg (fun h => ht ⟨by linarith [h.1.2], by linarith [h.1.1]⟩),
          if_neg (fun h => ht h.1)]; ring

theorem power_refl (P : Params) (s : St) (i : In) :
    power s.refl.settled s.refl.settledTarget P.refl.target s.refl.theta i.refl.n1 i.refl.n2
      i.refl.n3 P.refl.tol P.refl.exitFactor P.refl.ffMin P.refl.kv P.refl.p P.refl.ksPos
      P.refl.ksNeg P.refl.maxPower P.refl.travelMin P.refl.travelMax
    = -power s.settled s.settledTarget P.target s.theta i.n1 i.n2 i.n3 P.tol P.exitFactor P.ffMin
      P.kv P.p P.ksPos P.ksNeg P.maxPower P.travelMin P.travelMax := by
  simp only [St.refl, In.refl, Params.refl]
  unfold power
  have hs := settledNext_refl s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor
  by_cases h : settledNext s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor
  · rw [if_pos (hs.2 h), if_pos h, ← neg_zero, output_refl, neg_zero]
  · rw [if_neg (fun h' => h (hs.1 h')), if_neg h]
    unfold measuredError
    rw [show -P.target - (-s.theta + -i.n1) = -(P.target - (s.theta + i.n1)) by ring,
      driveCommand_refl, output_refl]

theorem servo_refl (y u dt lag : ℝ) : servo (-y) (-u) dt lag = -servo y u dt lag := by
  unfold servo; split_ifs <;> ring

theorem effective_refl (y sP sN : ℝ) (hP : 0 ≤ sP) (hN : 0 ≤ sN) :
    effective (-y) sN sP = -effective y sP sN := by
  unfold effective
  rcases lt_trichotomy y 0 with h | h | h
  · rw [if_pos (show -y ≥ 0 by linarith), if_neg (show ¬ y ≥ 0 by linarith)]
    split_ifs <;> ring
  · subst h; simp [hP, hN]
  · rw [if_neg (show ¬ -y ≥ 0 by linarith), if_pos h.le, neg_neg]
    split_ifs <;> ring

theorem angle_refl (th eff k dt lo hi : ℝ) (h : lo ≤ hi) :
    angle (-th) (-eff) k dt (-hi) (-lo) = -angle th eff k dt lo hi := by
  unfold angle
  rw [show -th + k * -eff * dt = -(th + k * eff * dt) by ring, max_neg_neg, min_neg_neg, neg_inj]
  rcases le_total (th + k * eff * dt) lo with h1 | h1 <;> rcases le_total (th + k * eff * dt) hi with h2 | h2 <;>
    simp [h1, h2, max_eq_left, max_eq_right, min_eq_left, min_eq_right, h] <;> linarith

/-- The loop commutes with the mirror. -/
theorem step_refl (P : Params) (s : St) (i : In) (hP : P.stopMin ≤ P.stopMax)
    (hsP : 0 ≤ P.sPos) (hsN : 0 ≤ P.sNeg) :
    step P.refl s.refl i.refl = (step P s i).refl := by
  have hpow := power_refl P s i
  simp only [step, St.refl] at hpow ⊢
  simp only [Params.refl, In.refl] at hpow ⊢
  rw [hpow, servo_refl, effective_refl _ _ _ hsP hsN, angle_refl _ _ _ _ _ _ hP,
    settledTargetNext_refl]
  have hs := settledNext_refl s.settled s.settledTarget P.target s.theta i.n1 P.tol P.exitFactor
  simp only [St.mk.injEq, true_and, and_true]
  exact decide_eq_decide.mpr hs

theorem Params.Valid.refl {P : Params} (hP : P.Valid) : P.refl.Valid := by
  obtain ⟨t1, t2, t3, t4⟩ := hP.travel
  obtain ⟨s1, s2, s3, s4⟩ := hP.stops
  exact { k_pos := hP.k_pos, sPos_nonneg := hP.sNeg_nonneg, sNeg_nonneg := hP.sPos_nonneg,
          lag_nonneg := hP.lag_nonneg, noise_nonneg := hP.noise_nonneg, noise_lt := hP.noise_lt,
          tol_le := hP.tol_le, exit_ge := hP.exit_ge, exit_band := hP.exit_band,
          band_small := hP.band_small, p_nonneg := hP.p_nonneg, ksPos_nonneg := hP.ksNeg_nonneg,
          ksNeg_nonneg := hP.ksPos_nonneg, maxPower_pos := hP.maxPower_pos,
          maxPower_le := hP.maxPower_le,
          travel := ⟨by simp [Params.refl]; linarith, by simp [Params.refl]; linarith,
                     by simp [Params.refl]; linarith, by simp [Params.refl]; linarith⟩,
          stops := ⟨by simp [Params.refl]; linarith, by simp [Params.refl]; linarith,
                    by simp [Params.refl]; linarith, by simp [Params.refl]; linarith⟩ }

theorem In.Valid.refl {P : Params} {dtMax : ℝ} {i : In} (hi : i.Valid P dtMax) :
    i.refl.Valid P.refl dtMax :=
  { dt_pos := hi.dt_pos, dt_le := hi.dt_le,
    n1 := by simpa [In.refl, Params.refl, abs_neg] using hi.n1,
    n2 := by simpa [In.refl, Params.refl, abs_neg] using hi.n2,
    n3 := by simpa [In.refl, Params.refl, abs_neg] using hi.n3 }

theorem hold_refl {P : Params} {s : St} : Hold P.refl s.refl ↔ Hold P s := by
  unfold Hold
  simp only [Params.refl, St.refl]
  rw [show -P.target - -s.theta - P.k * P.lag * -s.servo
      = -(P.target - s.theta - P.k * P.lag * s.servo) by ring,
    show -P.target - -s.settledTarget = -(P.target - s.settledTarget) by ring,
    show -P.target - -s.theta = -(P.target - s.theta) by ring,
    abs_neg, abs_neg, abs_neg, abs_neg]
  constructor <;> rintro ⟨h1, h2, h3, h4, h5, h6, h7⟩ <;>
    exact ⟨h1, h2, h3, h4, h5, by linarith, by linarith⟩

/-- Approaching from below, each loop is either in the hold set or still approaching with the
ranking measure down by `delta` per loop so far. -/
theorem approach_up_run {P : Params} {a b m dtMax dtMin : ℝ} (hP : P.Valid)
    (hW : Witness P dtMax a b m) (traj : ℕ → St) (inp : ℕ → In)
    (hin : ∀ n, (inp n).Valid P dtMax) (hmin : ∀ n, dtMin ≤ (inp n).dt)
    (hstep : ∀ n, traj (n + 1) = step P (traj n) (inp n))
    (h0 : ApproachUp P a b (traj 0)) :
    ∀ n, Hold P (traj n) ∨ (ApproachUp P a b (traj n) ∧
      potentialUp P m (traj n) ≤ potentialUp P m (traj 0) - n * (P.k * dtMin * (m / 2))) := by
  intro n
  induction n with
  | zero => right; exact ⟨h0, by simp⟩
  | succ n ih =>
    rw [hstep n]
    rcases ih with hh | ⟨ha, hphi⟩
    · left; exact hold_step hP (hin n) hh
    · have hfar := ha.2.1
      obtain ⟨hn1a, hn1b⟩ := abs_le.mp (hin n).n1
      rcases le_or_gt (P.target - ((traj n).theta + (inp n).n1)) P.tol with hle | hgt
      · left
        exact approach_up_settle hP hW (hin n) ha (abs_le.mpr ⟨by linarith, hle⟩)
      · right
        obtain ⟨ha', hphi'⟩ := approach_up_drive hP hW (hin n) ha hgt
        refine ⟨ha', ?_⟩
        have hd : P.k * dtMin * (m / 2) ≤ P.k * (inp n).dt * (m / 2) := by
          have := hW.m_pos
          have := mul_le_mul_of_nonneg_left (hmin n) hP.k_pos.le
          nlinarith
        push_cast
        nlinarith

/-- **Theorem (convergence).** For a plant in the box, gains meeting the conditions for both
directions, and any loop periods and encoder errors within their bounds: a turret that starts
settled, or approaching a still target from either side, is in the hold set from some loop on, and
stays there: settled, and aimed within the exit band. -/
theorem converges {P : Params} {aU bU mU aD bD mD dtMax dtMin : ℝ} (hP : P.Valid)
    (hWU : Witness P dtMax aU bU mU) (hWD : Witness P.refl dtMax aD bD mD) (hdtMin : 0 < dtMin)
    (traj : ℕ → St) (inp : ℕ → In) (hin : ∀ n, (inp n).Valid P dtMax)
    (hmin : ∀ n, dtMin ≤ (inp n).dt) (hstep : ∀ n, traj (n + 1) = step P (traj n) (inp n))
    (hstart : Hold P (traj 0) ∨ ApproachUp P aU bU (traj 0) ∨
      ApproachUp P.refl aD bD (traj 0).refl) :
    ∃ N : ℕ, ∀ n, N ≤ n → Hold P (traj n) := by
  -- From below: the ranking measure is bounded below, so the approach ends.
  have up : ∀ (Q : Params) (a b m : ℝ), Q.Valid → Witness Q dtMax a b m →
      ∀ (tr : ℕ → St) (ip : ℕ → In), (∀ n, (ip n).Valid Q dtMax) → (∀ n, dtMin ≤ (ip n).dt) →
      (∀ n, tr (n + 1) = step Q (tr n) (ip n)) → ApproachUp Q a b (tr 0) →
      ∃ N : ℕ, ∀ n, N ≤ n → Hold Q (tr n) := by
    intro Q a b m hQ hW tr ip hip hipm htr h0
    have hrun := approach_up_run (m := m) hQ hW tr ip hip hipm htr h0
    have hdel : 0 < Q.k * dtMin * (m / 2) := by
      have := hW.m_pos; have := hQ.k_pos; positivity
    obtain ⟨N, hN⟩ := exists_nat_gt (potentialUp Q m (tr 0) / (Q.k * dtMin * (m / 2)))
    refine ⟨N, fun n hn => ?_⟩
    rcases hrun n with hh | ⟨ha, hphi⟩
    · exact hh
    · exfalso
      have hpos := potentialUp_nonneg (m := m) hQ ha
      have hnN : (N : ℝ) ≤ n := by exact_mod_cast hn
      have : potentialUp Q m (tr 0) < n * (Q.k * dtMin * (m / 2)) := by
        rw [div_lt_iff₀ hdel] at hN; nlinarith
      linarith
  rcases hstart with hh | hu | hd
  · refine ⟨0, fun n _ => ?_⟩
    induction n with
    | zero => exact hh
    | succ n ih => rw [hstep n]; exact hold_step hP (hin n) (ih (Nat.zero_le _))
  · exact up P aU bU mU hP hWU traj inp hin hmin hstep hu
  · -- From above: the mirror image approaches from below.
    obtain ⟨N, hN⟩ := up P.refl aD bD mD hP.refl hWD (fun n => (traj n).refl)
      (fun n => (inp n).refl) (fun n => (hin n).refl) (fun n => hmin n)
      (fun n => by
        show (traj (n + 1)).refl = step P.refl (traj n).refl (inp n).refl
        rw [hstep n, step_refl P (traj n) (inp n)
          (by obtain ⟨_, h1, h2, _⟩ := hP.stops; obtain ⟨_, h3, h4, _⟩ := hP.travel; linarith)
          hP.sPos_nonneg hP.sNeg_nonneg])
      hd
    exact ⟨N, fun n hn => hold_refl.1 (hN n hn)⟩

end TurretStep
end
