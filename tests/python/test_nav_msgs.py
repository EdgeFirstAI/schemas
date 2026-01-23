"""Tests for nav_msgs module."""

import pytest

from edgefirst.schemas import builtin_interfaces, geometry_msgs, nav_msgs


class TestOdometry:
    """Tests for Odometry message."""

    def test_odometry_creation(self, sample_odometry):
        """Test Odometry structure."""
        assert sample_odometry.child_frame_id == "base_link"
        assert sample_odometry.pose is not None
        assert sample_odometry.twist is not None

    def test_odometry_serialize_deserialize(self, sample_odometry):
        """Test CDR serialization roundtrip."""
        data = sample_odometry.serialize()
        restored = nav_msgs.Odometry.deserialize(data)
        assert restored.child_frame_id == sample_odometry.child_frame_id
        assert restored.pose.pose.position.x == pytest.approx(
            sample_odometry.pose.pose.position.x
        )
        assert restored.twist.twist.linear.x == pytest.approx(
            sample_odometry.twist.twist.linear.x
        )

    def test_odometry_covariance(self, sample_header):
        """Test Odometry with specific covariance values."""
        pose_cov = [0.01 * i for i in range(36)]
        twist_cov = [0.02 * i for i in range(36)]

        odom = nav_msgs.Odometry(
            header=sample_header,
            child_frame_id="odom",
            pose=geometry_msgs.PoseWithCovariance(
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Point(x=1.0, y=2.0, z=0.0),
                    orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
                ),
                covariance=pose_cov,
            ),
            twist=geometry_msgs.TwistWithCovariance(
                twist=geometry_msgs.Twist(
                    linear=geometry_msgs.Vector3(x=0.5, y=0.0, z=0.0),
                    angular=geometry_msgs.Vector3(x=0.0, y=0.0, z=0.1),
                ),
                covariance=twist_cov,
            ),
        )

        data = odom.serialize()
        restored = nav_msgs.Odometry.deserialize(data)
        assert len(restored.pose.covariance) == 36
        assert restored.pose.covariance[1] == pytest.approx(0.01)
        assert restored.twist.covariance[1] == pytest.approx(0.02)


class TestPath:
    """Tests for Path message."""

    def test_path_creation(self, sample_path):
        """Test Path structure."""
        assert len(sample_path.poses) > 0
        assert sample_path.poses[0].pose.position is not None

    def test_path_serialize_deserialize(self, sample_path):
        """Test CDR serialization roundtrip."""
        data = sample_path.serialize()
        restored = nav_msgs.Path.deserialize(data)
        assert len(restored.poses) == len(sample_path.poses)
        assert restored.poses[0].pose.position.x == pytest.approx(
            sample_path.poses[0].pose.position.x
        )

    def test_path_trajectory(self, sample_header):
        """Test Path representing a trajectory."""
        poses = [
            geometry_msgs.PoseStamped(
                header=sample_header,
                pose=geometry_msgs.Pose(
                    position=geometry_msgs.Point(
                        x=float(i) * 0.1,
                        y=float(i) * 0.05,
                        z=0.0,
                    ),
                    orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
                ),
            )
            for i in range(100)
        ]
        path = nav_msgs.Path(header=sample_header, poses=poses)

        data = path.serialize()
        restored = nav_msgs.Path.deserialize(data)
        assert len(restored.poses) == 100
        assert restored.poses[50].pose.position.x == pytest.approx(5.0)

    def test_path_empty(self, sample_header):
        """Test Path with no poses."""
        path = nav_msgs.Path(header=sample_header, poses=[])

        data = path.serialize()
        restored = nav_msgs.Path.deserialize(data)
        assert len(restored.poses) == 0


class TestOccupancyGrid:
    """Tests for OccupancyGrid message."""

    def test_occupancy_grid_creation(self, sample_occupancy_grid):
        """Test OccupancyGrid structure."""
        assert sample_occupancy_grid.info.width == 100
        assert sample_occupancy_grid.info.height == 100
        assert len(sample_occupancy_grid.data) == 100 * 100

    def test_occupancy_grid_serialize_deserialize(self, sample_occupancy_grid):
        """Test CDR serialization roundtrip."""
        data = sample_occupancy_grid.serialize()
        restored = nav_msgs.OccupancyGrid.deserialize(data)
        assert restored.info.width == sample_occupancy_grid.info.width
        assert restored.info.height == sample_occupancy_grid.info.height
        assert len(restored.data) == len(sample_occupancy_grid.data)

    def test_occupancy_grid_values(self, sample_header):
        """Test OccupancyGrid with specific occupancy values."""
        width, height = 10, 10
        # Create grid with known pattern: -1=unknown, 0=free, 100=occupied
        grid_data = []
        for j in range(height):
            for i in range(width):
                if i == 0 or i == width - 1 or j == 0 or j == height - 1:
                    grid_data.append(100)  # Occupied boundary
                elif i == 5 and j == 5:
                    grid_data.append(-1)  # Unknown center
                else:
                    grid_data.append(0)  # Free space

        grid = nav_msgs.OccupancyGrid(
            header=sample_header,
            info=nav_msgs.MapMetaData(
                map_load_time=builtin_interfaces.Time(sec=0, nanosec=0),
                resolution=0.05,
                width=width,
                height=height,
                origin=geometry_msgs.Pose(
                    position=geometry_msgs.Point(x=0.0, y=0.0, z=0.0),
                    orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
                ),
            ),
            data=grid_data,
        )

        data = grid.serialize()
        restored = nav_msgs.OccupancyGrid.deserialize(data)
        assert restored.data[0] == 100  # Corner occupied
        assert restored.data[55] == -1  # Center unknown
        assert restored.data[11] == 0  # Interior free

    @pytest.mark.slow
    def test_occupancy_grid_large(self, sample_header):
        """Test OccupancyGrid with large map (1000x1000)."""
        width, height = 1000, 1000
        grid_data = [0] * (width * height)

        grid = nav_msgs.OccupancyGrid(
            header=sample_header,
            info=nav_msgs.MapMetaData(
                map_load_time=builtin_interfaces.Time(sec=0, nanosec=0),
                resolution=0.01,  # 1cm resolution
                width=width,
                height=height,
                origin=geometry_msgs.Pose(
                    position=geometry_msgs.Point(x=-5.0, y=-5.0, z=0.0),
                    orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
                ),
            ),
            data=grid_data,
        )

        data = grid.serialize()
        assert len(data) > 1_000_000  # Over 1MB
        restored = nav_msgs.OccupancyGrid.deserialize(data)
        assert restored.info.width == 1000
        assert restored.info.height == 1000


class TestMapMetaData:
    """Tests for MapMetaData message."""

    def test_map_metadata_creation(self):
        """Test MapMetaData structure."""
        metadata = nav_msgs.MapMetaData(
            map_load_time=builtin_interfaces.Time(sec=100, nanosec=0),
            resolution=0.05,
            width=200,
            height=200,
            origin=geometry_msgs.Pose(
                position=geometry_msgs.Point(x=-5.0, y=-5.0, z=0.0),
                orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
            ),
        )
        assert metadata.resolution == pytest.approx(0.05)
        assert metadata.width == 200
        assert metadata.height == 200

    def test_map_metadata_serialize_deserialize(self):
        """Test CDR serialization roundtrip."""
        metadata = nav_msgs.MapMetaData(
            map_load_time=builtin_interfaces.Time(sec=50, nanosec=123),
            resolution=0.1,
            width=100,
            height=150,
            origin=geometry_msgs.Pose(
                position=geometry_msgs.Point(x=-2.5, y=-3.75, z=0.0),
                orientation=geometry_msgs.Quaternion(x=0, y=0, z=0, w=1),
            ),
        )

        data = metadata.serialize()
        restored = nav_msgs.MapMetaData.deserialize(data)
        assert restored.resolution == pytest.approx(0.1)
        assert restored.width == 100
        assert restored.height == 150
        assert restored.origin.position.x == pytest.approx(-2.5)


class TestGridCells:
    """Tests for GridCells message."""

    def test_grid_cells_creation(self, sample_header):
        """Test GridCells structure."""
        cells = nav_msgs.GridCells(
            header=sample_header,
            cell_width=0.05,
            cell_height=0.05,
            cells=[
                geometry_msgs.Point(x=0.0, y=0.0, z=0.0),
                geometry_msgs.Point(x=0.05, y=0.0, z=0.0),
                geometry_msgs.Point(x=0.0, y=0.05, z=0.0),
            ],
        )
        assert cells.cell_width == pytest.approx(0.05)
        assert len(cells.cells) == 3

    def test_grid_cells_serialize_deserialize(self, sample_header):
        """Test CDR serialization roundtrip."""
        cells = nav_msgs.GridCells(
            header=sample_header,
            cell_width=0.1,
            cell_height=0.1,
            cells=[
                geometry_msgs.Point(x=1.0, y=2.0, z=0.0),
                geometry_msgs.Point(x=1.1, y=2.0, z=0.0),
            ],
        )

        data = cells.serialize()
        restored = nav_msgs.GridCells.deserialize(data)
        assert restored.cell_width == pytest.approx(0.1)
        assert len(restored.cells) == 2
        assert restored.cells[0].x == pytest.approx(1.0)
