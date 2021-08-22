import pytest
from wind_tunnel2 import imu_init, read_imu, clear_imu, get_imu

# Dummy exception for mocking
EXCEPT = Exception('mocked')

# Mock qwiic_icm20948 class for the imu
@pytest.fixture
def mock_imu_obj(mocker):
    return mocker.patch('qwiic_icm20948.QwiicIcm20948')

#=================================================================================================
# TEST imu_init()
#=================================================================================================
class TestImuInit:
    @pytest.fixture
    def imu_init_success_setup(self):
        clear_imu()

        yield

        clear_imu()

    @pytest.fixture
    def imu_init_exception_setup(self, mock_imu_obj):
        imu_init()

        yield

        clear_imu()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~        
    def test_imu_init_success_connected(self, imu_init_success_setup, mock_imu_obj):
        mock_imu_obj.return_value.connected = True
        imu_init()

        result = get_imu()

        assert result is not None

    def test_imu_init_success_not_connected(self, imu_init_success_setup, mock_imu_obj):
        mock_imu_obj.return_value.connected = False
        imu_init()

        result = get_imu()

        assert result is not None

    def test_imu_init_qwiic_exception(self, imu_init_exception_setup, mock_imu_obj):
        mock_imu_obj.side_effect = EXCEPT
        imu_init()

        result = get_imu()

        assert result is None

    def test_imu_init_begin_exception(self, imu_init_exception_setup, mock_imu_obj):
        mock_imu_obj.return_value.connected = True
        mock_imu_obj.return_value.begin.side_effect = EXCEPT
        imu_init()

        result = get_imu()

        assert result is None

#=================================================================================================
# TEST read_imu()
#=================================================================================================
class TestReadImu:
    failed_value = [None for _ in range(9)]

    @pytest.fixture
    def mock_imu_init(self, mocker):
        return mocker.patch('wind_tunnel2.imu_init')

    @pytest.fixture(autouse=True)
    def read_imu_setup(self, mock_imu_obj):
        imu_init()

        yield

        clear_imu()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_read_imu_success_data_ready(self, mock_imu_obj):
        mock_imu_obj.return_value.dataReady.return_value = True

        result = read_imu()

        assert result != self.failed_value

    def test_read_imu_success_not_data_ready(self, mock_imu_obj):
        mock_imu_obj.return_value.dataReady.return_value = False

        result = read_imu()

        assert result == self.failed_value

    def test_read_imu_data_ready_exception(self, mock_imu_init, mock_imu_obj):
        mock_imu_obj.return_value.dataReady.side_effect = EXCEPT

        result = read_imu()

        assert result == self.failed_value

    def test_read_imu_get_agmt_exception(self, mock_imu_init, mock_imu_obj):
        mock_imu_obj.return_value.getAgmt.side_effect = EXCEPT

        result = read_imu()

        assert result == self.failed_value
