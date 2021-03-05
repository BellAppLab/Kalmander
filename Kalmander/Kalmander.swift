import Foundation
import Surge
import class CoreLocation.CLLocation
import struct CoreLocation.CLLocationCoordinate2D


/// The dimension M of the state vector.
private let stateMDimension = 6

/// The dimension N of the state vector.
private let stateNDimension = 1

/// Acceleration variance magnitude for GPS
/// =======================================
/// **Sigma** value is  value for Acceleration Noise Magnitude Matrix (Qt).
/// Recommended value for **sigma** is 0.0625, this value is optimal for GPS problem,
/// it was concluded by researches.
private let sigma = 0.0625


/// Value for Sensor Noise Covariance Matrix
/// ========================================
/// Default value is 29.0, this is the recommended value for the GPS problem, with this value filter provides optimal accuracy.
/// This value can be adjusted depending on the needs, the higher value
/// of **rVaule** variable will give greater roundness trajectories, and vice versa.
public var rValue: Double = 29.0


//private typealias Dimensions = (rows: Int, columns: Int)
//
//private extension Matrix
//{
//    static func map(_ dimensions: Dimensions, _ closure: (_ row: Int, _ column: Int) -> Scalar) -> Matrix<Scalar> {
//        Matrix((0..<dimensions.rows).map { row in
//            (0..<dimensions.columns).map { column in
//                closure(row, column)
//            }
//        })
//    }
//}
//
//private extension Matrix where Scalar == Double
//{
//
//    static func identity(_ dimensions: Dimensions) -> Matrix<Scalar> {
//        map(dimensions) { (r, c) -> Scalar in
//            guard r == c else { return 0 }
//            return 1
//        }
//    }
//}


public struct Kalmander
{
    //MARK: - HCKalmanAlgorithm properties
    /// Previous State Vector
    /// =====================
    /// **Previous State Vector** is mathematical representation of previous state of Kalman Algorithm.
    private var xk1: Matrix<Double>


    /// Covariance Matrix for Previous State
    /// ====================================
    /// **Covariance Matrix for Previous State** is mathematical representation of covariance matrix for previous state of Kalman Algorithm.
    private var Pk1: Matrix<Double>


    /// Prediction Step Matrix
    /// ======================
    /// **Prediction Step Matrix (A)** is mathematical representation of prediction step of Kalman Algorithm.
    /// Prediction Matrix gives us our next state. It takes every point in our original estimate and moves it to a new predicted location,
    /// which is where the system would move if that original estimate was the right one.
    private var A: Matrix<Double>


    /// Acceleration Noise Magnitude Matrix
    /// ===================================
    /// **Acceleration Noise Magnitude Matrix (Qt)** is mathematical representation of external uncertainty of Kalman Algorithm.
    /// The uncertainty associated can be represented with the “world” (i.e. things we aren’t keeping track of)
    /// by adding some new uncertainty after every prediction step.
    private var Qt: Matrix<Double>


    /// Sensor Noise Covariance Matrix
    /// ==============================
    /// **Sensor Noise Covariance Matrix (R)** is mathematical representation of sensor noise of Kalman Algorithm.
    /// Sensors are unreliable, and every state in our original estimate might result in a range of sensor readings.
    private var R: Matrix<Double>

    /// Measured State Vector
    /// =====================
    /// **Measured State Vector (zt)** is mathematical representation of measuerd state vector of Kalman Algorithm.
    /// Value of this variable was readed from sensor, this is mean value to the reading we observed.
    private var zt: Matrix<Double>

    /// Time of last measurement
    /// ========================
    /// This time is used for calculating the time interval between previous and last measurements
    private var previousMeasureTime: Date

    /// Previous State Location
    private var previousLocation: CLLocation


    //MARK: - HCKalmanAlgorithm initialization

    /// Initialization of Kalman Algorithm Constructor
    /// ==============================================
    /// - parameters:
    ///   - initialLocation: this is CLLocation object which represent initial location
    ///                      at the moment when algorithm start
    public init(initialLocation: CLLocation) throws
    {
        // Set Previous State Matrix
        // xk1 -> [ initial_lat  lat_velocity = 0.0  initial_lon  lon_velocity = 0.0 initial_altitude altitude_velocity = 0.0 ]
        let xk1 = Matrix(rows: stateMDimension, columns: stateNDimension) { _, c -> Double in
            switch c {
            case 0: return initialLocation.coordinate.latitude
            case 2: return initialLocation.coordinate.longitude
            case 4: return initialLocation.altitude
            default: return 0.0
            }
        }

        // Set initial Covariance Matrix for Previous State
        let Pk1 = Matrix(rows: stateMDimension, columns: stateMDimension, repeatedValue: 0.0)

        // Prediction Step Matrix initialization
        let A = Matrix<Double>.identity(size: stateMDimension)

        let Qt = Matrix(rows: stateMDimension, columns: stateMDimension, repeatedValue: 0.0)

        // Sensor Noise Covariance Matrix initialization
        let R = Matrix(rows: stateMDimension, columns: stateMDimension) { r, c -> Double in
            guard r == c else { return 0 }
            return rValue
        }

        let zt = Matrix(rows: stateMDimension, columns: stateMDimension, repeatedValue: 0.0)

        self.init(initialLocation, xk1, Pk1, A, Qt, R, zt)
    }

    private init(
        _ previousLocation: CLLocation,
        _ xk1: Matrix<Double>,
        _ Pk1: Matrix<Double>,
        _ A: Matrix<Double>,
        _ Qt: Matrix<Double>,
        _ R: Matrix<Double>,
        _ zt: Matrix<Double>,
        _ previousMeasureTime: Date? = nil
    ) {
        // Set initial location
        self.previousLocation = previousLocation

        // Set timestamp for start of measuring
        self.previousMeasureTime = previousMeasureTime ?? previousLocation.timestamp

        self.xk1 = xk1
        self.Pk1 = Pk1
        self.A = A
        self.Qt = Qt
        self.R = R
        self.zt = zt
    }

    /// Process Current Location
    /// ========================
    ///  This function is a main. **processState** will be processed current location of user by Kalman Filter
    ///  based on previous state and other parameters, and it returns corrected location
    /// - parameters:
    ///   - currentLocation: this is CLLocation object which represent current location returned by GPS.
    ///                      **currentLocation** is real position of user, and it will be processed by Kalman Filter.
    /// - returns: CLLocation object with corrected latitude, longitude and altitude values

    public mutating func processState(currentLocation: CLLocation) -> CLLocation
    {
        // Set current timestamp
        let newMeasureTime = currentLocation.timestamp

        // Convert measure times to seconds
        let newMeasureTimeSeconds = newMeasureTime.timeIntervalSince1970
        let lastMeasureTimeSeconds = previousMeasureTime.timeIntervalSince1970

        // Calculate timeInterval between last and current measure
        let timeInterval = newMeasureTimeSeconds - lastMeasureTimeSeconds

        // Calculate and set Prediction Step Matrix based on new timeInterval value
        let A = Matrix(rows: stateMDimension, columns: stateMDimension) { r, c -> Double in
            if r == c {
                return 1
            }
            if r % 2 == 0, c == r + 1 {
                return timeInterval
            }
            return 0
        }

        // Parts of Acceleration Noise Magnitude Matrix
        let part1 = sigma * (pow(timeInterval, 4.0) / 4.0)
        let part2 = sigma * (pow(timeInterval, 3.0) / 2.0)
        let part3 = sigma * pow(timeInterval, 2.0)

        // Calculate and set Acceleration Noise Magnitude Matrix based on new timeInterval and sigma values
        let Qt = Matrix(rows: stateMDimension, columns: stateMDimension) { r, c -> Double in
            switch (r, c) {
            case (0, 0): return part1
            case (0, 1): return part2
            case (1, 0): return part2
            case (1, 1): return part3
            case (2, 2): return part1
            case (2, 3): return part2
            case (3, 2): return part2
            case (3, 3): return part3
            case (4, 4): return part1
            case (4, 5): return part2
            case (5, 4): return part2
            case (5, 5): return part3
            default: return 0
            }
        }

        // Calculate velocity components
        // This is value of velocity between previous and current location.
        // Distance traveled from the previous to the current location divided by timeInterval between two measurement.
        let velocityXComponent = (previousLocation.coordinate.latitude - currentLocation.coordinate.latitude) / timeInterval
        let velocityYComponent = (previousLocation.coordinate.longitude - currentLocation.coordinate.longitude) / timeInterval
        let velocityZComponent = (previousLocation.altitude - currentLocation.altitude) / timeInterval

        // Set Measured State Vector; current latitude, longitude, altitude and latitude velocity, longitude velocity and altitude velocity
        let zt = Matrix([[currentLocation.coordinate.latitude], [velocityXComponent], [currentLocation.coordinate.longitude], [velocityYComponent], [currentLocation.altitude], [velocityZComponent]])

        /// Here happens the whole mathematics related to Kalman Filter. Here is the essence.
        /// The algorithm consists of two parts - Part of Prediction and Part of Update State
        ///
        /// Prediction part performs the prediction of the next state based on previous state, prediction matrix (A) and takes into consideration
        /// external uncertainty factor (Qt). It returns predicted state and covariance matrix -> xk, Pk
        ///
        /// Next step is Update part. It combines predicted state with sensor measurement. Update part first calculate Kalman gain (Kt).
        /// Kalman gain takes into consideration sensor noice. Next based on this value, value of predicted state and value of measurement,
        /// algorithm can calculate new state, and function return corrected latitude, longitude and altitude values in CLLocation object.
        let xk = A * xk1
        let Pk = ((A * Pk1) * transpose(A)) + Qt

        let tmp = Pk + R

        // Kalman gain (Kt)
        let Kt = Pk * inv(tmp)

        let xt = xk + (Kt * (zt - xk))
        let Pt = (Matrix.identity(size: stateMDimension) - Kt) * Pk

        let lat = xt[0,0]
        let lon = xt[2,0]
        let altitude = xt[4,0]

        let kalmanCLLocation: CLLocation
        if #available(iOS 13.4, *) {
            kalmanCLLocation = CLLocation(
                coordinate: CLLocationCoordinate2D(latitude: lat, longitude: lon),
                altitude: altitude,
                horizontalAccuracy: currentLocation.horizontalAccuracy,
                verticalAccuracy: currentLocation.verticalAccuracy,
                course: currentLocation.course,
                courseAccuracy: currentLocation.courseAccuracy,
                speed: currentLocation.speed,
                speedAccuracy: currentLocation.speedAccuracy,
                timestamp: newMeasureTime
            )
        } else {
            kalmanCLLocation = CLLocation(
                coordinate: CLLocationCoordinate2D(latitude: lat, longitude: lon),
                altitude: altitude,
                horizontalAccuracy: currentLocation.horizontalAccuracy,
                verticalAccuracy: currentLocation.verticalAccuracy,
                timestamp: newMeasureTime
            )
        }

        // Set previous Location and Measure Time for next step of processState function.
        self = Kalmander(currentLocation, xt, Pt, A, Qt, R, zt, newMeasureTime)

        // Return value of kalmanFilter
        return kalmanCLLocation
    }
}
