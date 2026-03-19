package frc.robot.subsystems.indexer;

import frc.robot.constants.IndexerConstants;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputsAutoLogged;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs = new IndexerIOInputsAutoLogged();

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    public void intake() {
        io.setRollersOpenLoop(IndexerConstants.rollersSpeed);
    }

    public void outtake() {
        io.setFeederOpenLoop(-IndexerConstants.feederSpeed);
        io.setShooterWheelsOpenLoop(-IndexerConstants.shooterWheelsSpeed);
        io.setTurretWheelsOpenLoop(-IndexerConstants.turretWheelsSpeed);
    }

    public void stopRollers() {
        io.setRollersOpenLoop(0.0);
    }

    public void indexBoth() {
        io.setShooterWheelsOpenLoop(IndexerConstants.shooterWheelsSpeed);
        io.setTurretWheelsOpenLoop(IndexerConstants.turretWheelsSpeed);
        io.setFeederOpenLoop(IndexerConstants.feederSpeed);
    }

    public void indexShooter() {
        io.setShooterWheelsOpenLoop(IndexerConstants.shooterWheelsSpeed);
        io.setTurretWheelsOpenLoop(-IndexerConstants.turretWheelsSpeed);
        io.setFeederOpenLoop(IndexerConstants.feederSpeed);
    }

    public void indexTurret() {
        io.setShooterWheelsOpenLoop(-IndexerConstants.shooterWheelsSpeed);
        io.setTurretWheelsOpenLoop(IndexerConstants.turretWheelsSpeed);
        io.setFeederOpenLoop(IndexerConstants.feederSpeed);
    }

    public void stopIndexer() {
        io.setShooterWheelsOpenLoop(0.0);
        io.setTurretWheelsOpenLoop(0.0);
        io.setFeederOpenLoop(0.0);
    }
}
