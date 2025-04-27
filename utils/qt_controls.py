"""
PyQt5-based control panel for Kirigami simulation.

This module provides a resizable window with sliders that interact with
the PyBullet simulation, allowing for more precise control especially
when dealing with a large number of tiles.
"""
import sys
import PyQt5
from PyQt5.QtWidgets import (QApplication, QMainWindow, QSlider, QLabel, 
                            QVBoxLayout, QPushButton, QWidget, QHBoxLayout,
                            QSpinBox, QGroupBox, QSplitter, QSizePolicy)
from PyQt5.QtCore import Qt, QTimer

class KirigamiControlPanel(QMainWindow):
    """
    A PyQt5-based control panel that provides resizable sliders for the Kirigami simulation.
    This window works alongside the PyBullet GUI and provides more precise control.
    """
    def __init__(self, simulation_data, event_handler):
        """
        Initialize the control panel window.
        
        Args:
            simulation_data: Dictionary containing simulation data
            event_handler: The event handler instance from the PyBullet simulation
        """
        super().__init__()
        
        # Store references to simulation components
        self.simulation_data = simulation_data
        self.event_handler = event_handler
        
        # Set up the main window
        self.setWindowTitle("Kirigami Simulation Controls")
        self.setGeometry(100, 100, 400, 400)
        
        # Create the central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Create the tile selection group
        tile_selection_group = QGroupBox("Tile Selection")
        tile_layout = QVBoxLayout()
        
        # Create Tile 1 selection controls
        tile1_layout = QHBoxLayout()
        self.tile1_label = QLabel("Tile 1 Index:")
        self.tile1_slider = QSlider(Qt.Horizontal)
        self.tile1_slider.setMinimum(0)
        self.tile1_slider.setMaximum(len(self.simulation_data['bricks'])-1)
        self.tile1_slider.setValue(0)
        self.tile1_spinbox = QSpinBox()
        self.tile1_spinbox.setMinimum(0)
        self.tile1_spinbox.setMaximum(len(self.simulation_data['bricks'])-1)
        
        tile1_layout.addWidget(self.tile1_label)
        tile1_layout.addWidget(self.tile1_slider)
        tile1_layout.addWidget(self.tile1_spinbox)
        
        # Create Tile 2 selection controls
        tile2_layout = QHBoxLayout()
        self.tile2_label = QLabel("Tile 2 Index:")
        self.tile2_slider = QSlider(Qt.Horizontal)
        self.tile2_slider.setMinimum(0)
        self.tile2_slider.setMaximum(len(self.simulation_data['bricks'])-1)
        self.tile2_slider.setValue(1)
        self.tile2_spinbox = QSpinBox()
        self.tile2_spinbox.setMinimum(0)
        self.tile2_spinbox.setMaximum(len(self.simulation_data['bricks'])-1)
        self.tile2_spinbox.setValue(1)
        
        tile2_layout.addWidget(self.tile2_label)
        tile2_layout.addWidget(self.tile2_slider)
        tile2_layout.addWidget(self.tile2_spinbox)
        
        # Add remove constraint button
        self.remove_constraint_button = QPushButton("Remove Constraint")
        
        # Add layouts to tile selection group
        tile_layout.addLayout(tile1_layout)
        tile_layout.addLayout(tile2_layout)
        tile_layout.addWidget(self.remove_constraint_button)
        tile_selection_group.setLayout(tile_layout)
        
        # Add the tile selection group to the main layout
        main_layout.addWidget(tile_selection_group)
        
        # Create the simulation control group
        sim_control_group = QGroupBox("Simulation Controls")
        sim_control_layout = QVBoxLayout()
        
        # Reset button
        self.reset_button = QPushButton("Reset Simulation")
        sim_control_layout.addWidget(self.reset_button)
        
        # Save vertices button
        self.save_button = QPushButton("Save Vertices")
        sim_control_layout.addWidget(self.save_button)
        
        # Toggle labels button
        self.toggle_labels_button = QPushButton("Toggle Labels")
        sim_control_layout.addWidget(self.toggle_labels_button)
        
        # Set the layout for the simulation control group
        sim_control_group.setLayout(sim_control_layout)
        
        # Add the simulation control group to the main layout
        main_layout.addWidget(sim_control_group)
        
        # Set up connections between controls
        self.setup_connections()
        
        # Set up timer to sync with PyBullet parameters
        self.sync_timer = QTimer(self)
        self.sync_timer.timeout.connect(self.sync_with_pybullet)
        self.sync_timer.start(100)  # Update 10 times per second
        
        # Show the window
        self.show()
        
    def setup_connections(self):
        """Set up signal-slot connections between UI elements"""
        # Connect slider and spinbox for Tile 1
        self.tile1_slider.valueChanged.connect(self.tile1_spinbox.setValue)
        self.tile1_spinbox.valueChanged.connect(self.tile1_slider.setValue)
        self.tile1_spinbox.valueChanged.connect(self.update_pybullet_tile1)
        
        # Connect slider and spinbox for Tile 2
        self.tile2_slider.valueChanged.connect(self.tile2_spinbox.setValue)
        self.tile2_spinbox.valueChanged.connect(self.tile2_slider.setValue)
        self.tile2_spinbox.valueChanged.connect(self.update_pybullet_tile2)
        
        # Connect buttons
        self.remove_constraint_button.clicked.connect(self.remove_constraint)
        self.reset_button.clicked.connect(self.reset_simulation)
        self.save_button.clicked.connect(self.save_vertices)
        self.toggle_labels_button.clicked.connect(self.toggle_labels)
    
    def update_pybullet_tile1(self, value):
        """Update the tile1 index value in the event handler"""
        if hasattr(self.event_handler, 'ui_controls'):
            # Store the value directly in the event handler
            self.event_handler.tile1_index = value
    
    def update_pybullet_tile2(self, value):
        """Update the tile2 index value in the event handler"""
        if hasattr(self.event_handler, 'ui_controls'):
            # Store the value directly in the event handler
            self.event_handler.tile2_index = value
    
    def sync_with_pybullet(self):
        """Sync PyQt controls with PyBullet parameters"""
        # Check if the PyBullet controls have been updated
        if hasattr(self.event_handler, 'ui_controls'):
            # Update slider maximums if number of bricks has changed
            if self.tile1_slider.maximum() != len(self.simulation_data['bricks'])-1:
                new_max = len(self.simulation_data['bricks'])-1
                self.tile1_slider.setMaximum(new_max)
                self.tile1_spinbox.setMaximum(new_max)
                self.tile2_slider.setMaximum(new_max)
                self.tile2_spinbox.setMaximum(new_max)
    
    def remove_constraint(self):
        """Trigger constraint removal in PyBullet"""
        # Use tile indices directly from the spinboxes
        tile1_idx = self.tile1_spinbox.value()
        tile2_idx = self.tile2_spinbox.value()
        
        # Call the event handler's method to remove constraint
        if hasattr(self.event_handler, 'remove_constraint_between_tiles'):
            self.event_handler.remove_constraint_between_tiles(tile1_idx, tile2_idx)
    
    def reset_simulation(self):
        """Trigger simulation reset in PyBullet"""
        # Use the existing reset mechanism in the event handler
        if hasattr(self.event_handler, 'ui_controls'):
            # Call the reset method directly if available
            if hasattr(self.event_handler, 'reset_simulation'):
                self.event_handler.reset_simulation()
            else:
                # Otherwise set a flag that will be checked during simulation steps
                self.event_handler.reset_triggered = True
    
    def save_vertices(self):
        """Trigger vertex saving in PyBullet"""
        # Call the save method directly if available
        if hasattr(self.event_handler, 'save_vertex_locations'):
            self.event_handler.save_vertex_locations()
        # Otherwise set a flag that will be checked during simulation steps
        elif hasattr(self.event_handler, 'ui_controls'):
            self.event_handler.save_vertices_triggered = True
    
    def toggle_labels(self):
        """Toggle tile labels visibility in PyBullet"""
        # Toggle the label visibility in the event handler
        if hasattr(self.event_handler, 'labels_visible'):
            self.event_handler.labels_visible = not self.event_handler.labels_visible
            
            # Update labels based on new visibility setting
            if self.event_handler.labels_visible:
                self.event_handler.add_labels_to_tiles()
            else:
                # Call the method to remove all labels if available
                if hasattr(self.event_handler, '_remove_all_labels'):
                    self.event_handler._remove_all_labels()

def launch_qt_controls(simulation_data, event_handler):
    """
    Launch the PyQt5 control panel.
    
    This function should be called from the main simulation loop after
    the event handler has been created and UI controls set up.
    
    Args:
        simulation_data: Dictionary containing simulation data
        event_handler: The event handler instance from the PyBullet simulation
    
    Returns:
        The QApplication instance for the PyQt5 app
    """
    # Create Qt application if it doesn't exist
    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    
    # Create control panel window
    control_panel = KirigamiControlPanel(simulation_data, event_handler)
    
    # Process Qt events without blocking the main PyBullet simulation
    app.processEvents()
    
    return app, control_panel