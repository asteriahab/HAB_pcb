import pytest
from wind_tunnel2.py import imu_init, read_imu, clear_imu, get_imu

# Dummy exception for mocking
EXCEPT = Exception('mocked')

# Mock qwiic_icm20948 class for the imu
@pytest.fixture(autouse=True)
def mock_imu_obj(mocker):
    return mocker.patch('qwiic_icm20948.QwiicIcm20948')

#=================================================================================================
# TEST imu_init()
#=================================================================================================
class TestImuInit:
    @pytest.fixture(autouse=True)
    def imu_init_failed_val(self):
        return None

    @pytest.fixture
    def imu_init_success_setup(self):
        clear_imu()

        yield

        clear_imu()

    @pytest.fixture
    def imu_init_exception_setup(self):
        imu_init()

        yield

        clear_imu()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_imu_init_success_connected(self, imu_init_success_setup):
        mock_imu_obj.return_value.connected = True
        imu_init()

        imu_init_result = get_imu()

        assert imu_init_result != imu_init_failed_val

    def test_imu_init_success_not_connected(self, imu_init_success_setup):
        mock_imu_obj.return_value.connected = False
        imu_init()

        imu_init_result = get_imu()

        assert imu_init_result != imu_init_failed_val

    def test_imu_init_qwiic_exception(self, imu_init_exception_setup):
        mocker.patch('qwiic_icm20948.QwiicIcm20948', return_value = EXCEPT)
        imu_init()

        imu_init_result = get_imu()

        assert imu_init_result == imu_init_failed_val

    def test_imu_init_begin_exception(self, imu_init_exception_setup):
        mock_imu_obj.return_value.begin.return_value = EXCEPT
        imu_init()

        imu_init_result = get_imu()

        assert imu_init_result == imu_init_failed_val

#=================================================================================================
# TEST read_imu()
#=================================================================================================
class TestReadImu:
    @pytest.fixture(autouse=True)
    def read_imu_failed_value(self):
        return [None for _ in range(9)]

    @pytest.fixture
    def mock_imu_init(self):
        return mocker.patch('wind_tunnel2.imu_init')

    @pytest.fixture(autouse=True)
    def read_imu_setup(self):
        imu_init()

        yield

        clear_imu()
    #~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    def test_read_imu_success_data_ready(self):
        mock_imu_obj.return_value_dataReady.return_value = True

        read_imu_result = read_imu()

        assert read_imu_result != read_imu_failed_value

    def test_read_imu_success_not_data_ready(self):
        mock_imu_obj.return_value_dataReady.return_value = False

        read_imu_result = read_imu()

        assert read_imu_result != read_imu_failed_value

    def test_read_imu_data_ready_exception(self, mock_imu_init):
        mock_imu_obj.return_value.dataReady.return_value = EXCEPT

        read_imu_result = read_imu()

        assert read_imu_result == read_imu_failed_value

    def test_read_imu_get_agmt_exception(self, mock_imu_init):
        mock_imu_obj.return_value.getAgmt.return_value = EXCEPT

        read_imu_result = read_imu()

        assert read_imu_result == read_imu_failed_value
