#!/usr/bin/env pwsh

# Kirigami Simulation Runner Script
# This script contains predefined parameter sets for running the kirigami simulation

# Function to run simulation with a specific configuration
function Show-Help {
    Write-Host "`nKirigami Simulation Runner"
    Write-Host "======================="
    Write-Host "`nAvailable commands:"
    Write-Host "  .\run_kirigami_sim.ps1              - List all configurations"
    Write-Host "  .\run_kirigami_sim.ps1 run <name>   - Run a specific configuration"
    Write-Host "  .\run_kirigami_sim.ps1 info <name>  - Show details about a configuration"
    Write-Host "  .\run_kirigami_sim.ps1 notes <name> - Add/update notes for a configuration"
    Write-Host "  .\run_kirigami_sim.ps1 help        - Show this help message"
    Write-Host "`n"
}

function Run-KirigamiSim {
    param(
        [string]$ConfigName,
        [string]$Action = "list"
    )
    # Configuration dictionary
    $configs = @{
        "base" = @{
            "name" = "Base Configuration - Normal Force 550"
            "description" = "Basic setup with normal force and medium damping"
            "command" = "python run_sim.py --vertices_file data/contracted_splitted_tessellation_w3_h3_vertices.txt --constraints_file data/splitted0510_tessellation_w3_h3_constraints.txt --force_type normal --force_magnitude 550 --brick_thickness 0.1 --angular_damping 2.5 --linear_damping 2.5"
            "notes" = "Initial test configuration"
            "results" = "Not tested yet"
        }
        "high-force" = @{
            "name" = "High Force Configuration"
            "description" = "Increased force magnitude (700) for testing structural limits"
            "command" = "python run_sim.py --vertices_file data/contracted_splitted_tessellation_w3_h3_vertices.txt --constraints_file data/splitted0510_tessellation_w3_h3_constraints.txt --force_type normal --force_magnitude 700 --brick_thickness 0.1 --angular_damping 2.5 --linear_damping 2.5"
            "notes" = "Testing structural stability under higher forces"
            "results" = "Not tested yet"
        }
        "low-damping" = @{
            "name" = "Low Damping Configuration"
            "description" = "Reduced damping values for more dynamic behavior"
            "command" = "python run_sim.py --vertices_file data/contracted_splitted_tessellation_w3_h3_vertices.txt --constraints_file data/splitted0510_tessellation_w3_h3_constraints.txt --force_type normal --force_magnitude 550 --brick_thickness 0.1 --angular_damping 1.0 --linear_damping 1.0"
            "notes" = "Testing system behavior with lower damping"
            "results" = "Not tested yet"
        }
        "shear" = @{
            "name" = "Shear Force Configuration"
            "description" = "Using shear force instead of normal force"
            "command" = "python run_sim.py --vertices_file data/contracted_splitted_tessellation_w3_h3_vertices.txt --constraints_file data/splitted0510_tessellation_w3_h3_constraints.txt --force_type shear --force_magnitude 550 --brick_thickness 0.1 --angular_damping 2.5 --linear_damping 2.5"
            "notes" = "Testing shear deformation behavior"
            "results" = "Not tested yet"
        }
    }    switch ($Action.ToLower()) {
        "run" {
            if ($configs.ContainsKey($ConfigName)) {
                Write-Host "Running configuration: $($configs[$ConfigName].name)"
                Write-Host "Description: $($configs[$ConfigName].description)"
                Write-Host "Command: $($configs[$ConfigName].command)"
                Write-Host "`nExecuting simulation...`n"
                
                Invoke-Expression $configs[$ConfigName].command
            } else {
                Write-Host "Configuration '$ConfigName' not found."
                Write-Host "Use the script without parameters to see available configurations."
            }
        }
        "info" {
            if ($configs.ContainsKey($ConfigName)) {
                $config = $configs[$ConfigName]
                Write-Host "`nConfiguration Details: $ConfigName"
                Write-Host "=========================="
                Write-Host "Name: $($config.name)"
                Write-Host "Description: $($config.description)"
                Write-Host "Notes: $($config.notes)"
                Write-Host "Results: $($config.results)"
                Write-Host "Command: $($config.command)"
                Write-Host "`n"
            } else {
                Write-Host "Configuration '$ConfigName' not found."
            }
        }
        "notes" {
            if ($configs.ContainsKey($ConfigName)) {
                Write-Host "Current notes for $($configs[$ConfigName].name):"
                Write-Host $configs[$ConfigName].notes
                Write-Host "`nEnter new notes (press Enter when done):"
                $newNotes = Read-Host
                $configs[$ConfigName].notes = $newNotes
                Write-Host "Notes updated successfully."
            } else {
                Write-Host "Configuration '$ConfigName' not found."
            }
        }
        "help" {
            Show-Help
        }
        default {
            Write-Host "`nAvailable Configurations:"
            Write-Host "======================"
            foreach ($key in $configs.Keys) {
                Write-Host "`n$key : $($configs[$key].name)"
                Write-Host "Description: $($configs[$key].description)"
            }
            Write-Host "`nUse 'help' to see all available commands."
        }
    }
}

# Parse command line arguments
if ($args.Count -eq 0) {
    Run-KirigamiSim "" "list"
} elseif ($args.Count -eq 1) {
    if ($args[0] -eq "help") {
        Show-Help
    } else {
        Run-KirigamiSim "" $args[0]
    }
} else {
    Run-KirigamiSim $args[1] $args[0]
}
