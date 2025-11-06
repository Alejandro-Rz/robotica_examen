// Configuración del robot
const ROBOT_CONFIG = {
    l1: 0.12,    // Longitud eslabón 1 en metros
    l2: 0.12,    // Longitud eslabón 2 en metros
    gripper: 0.02, // Longitud pinza en metros
    initialPos: { x: 0.14, y: 0.14 }
};

// Estado global del robot
let robotState = {
    q1: Math.PI / 2,  // θ1 = 90° en radianes
    q2: 0,            // θ2 = 0° en radianes
    currentPos: { x: 0.14, y: 0.14 },
    trajectory: []
};

// Elementos del DOM
const canvas = document.getElementById('robotCanvas');
const ctx = canvas.getContext('2d');
const xdInput = document.getElementById('xd');
const ydInput = document.getElementById('yd');
const moveBtn = document.getElementById('moveBtn');
const homeBtn = document.getElementById('homeBtn');
const alertMessage = document.getElementById('alertMessage');
const currentPosSpan = document.getElementById('currentPos');
const theta1Span = document.getElementById('theta1');
const theta2Span = document.getElementById('theta2');

// Charts
let q1Chart, q2Chart;

// ==================== INICIALIZACIÓN ====================
function init() {
    setupEventListeners();
    initCharts();
    drawRobot();
    updateDisplay();
}

function setupEventListeners() {
    moveBtn.addEventListener('click', moveToPosition);
    homeBtn.addEventListener('click', goToHomePosition);
    
    // Actualizar en tiempo real mientras se escribe
    xdInput.addEventListener('input', validateInput);
    ydInput.addEventListener('input', validateInput);
}

function initCharts() {
    // Configuración común para las gráficas
    const chartConfig = {
        type: 'line',
        options: {
            responsive: true,
            maintainAspectRatio: false,
            scales: {
                x: {
                    title: { display: true, text: 'Tiempo (s)' },
                    min: 0,
                    max: 20
                },
                y: {
                    title: { display: true, text: 'Ángulo (rad)' }
                }
            },
            plugins: {
                legend: { display: false },
                title: { display: true, font: { size: 16 } }
            }
        }
    };

    // Gráfica para q1
    q1Chart = new Chart(document.getElementById('q1Chart'), {
        ...chartConfig,
        data: {
            labels: [],
            datasets: [{
                label: 'q₁(t)',
                data: [],
                borderColor: '#007bff',
                backgroundColor: 'rgba(0, 123, 255, 0.1)',
                tension: 0.4,
                fill: true
            }]
        },
        options: {
            ...chartConfig.options,
            plugins: {
                ...chartConfig.options.plugins,
                title: { ...chartConfig.options.plugins.title, text: 'Trayectoria Articular q₁ (θ₁)' }
            }
        }
    });

    // Gráfica para q2
    q2Chart = new Chart(document.getElementById('q2Chart'), {
        ...chartConfig,
        data: {
            labels: [],
            datasets: [{
                label: 'q₂(t)',
                data: [],
                borderColor: '#dc3545',
                backgroundColor: 'rgba(220, 53, 69, 0.1)',
                tension: 0.4,
                fill: true
            }]
        },
        options: {
            ...chartConfig.options,
            plugins: {
                ...chartConfig.options.plugins,
                title: { ...chartConfig.options.plugins.title, text: 'Trayectoria Articular q₂ (θ₂)' }
            }
        }
    });
}

// ==================== CINEMÁTICA INVERSA ====================
function inverseKinematics(x, y) {
    const l1 = ROBOT_CONFIG.l1;
    const l2 = ROBOT_CONFIG.l2;
    
    // Calcular q2 usando la ley de cosenos
    const D = (x * x + y * y - l1 * l1 - l2 * l2) / (2 * l1 * l2);
    
    // Verificar si el punto está alcanzable
    if (Math.abs(D) > 1) {
        return null; // Punto fuera del espacio de trabajo
    }
    
    // Dos soluciones posibles (usaremos la configuración "codo arriba")
    const q2 = Math.atan2(Math.sqrt(1 - D * D), D);
    
    // Calcular q1
    const q1 = Math.atan2(y, x) - Math.atan2(l2 * Math.sin(q2), l1 + l2 * Math.cos(q2));
    
    return { q1, q2 };
}

// ==================== VALIDACIÓN ESPACIO DE TRABAJO ====================
function isPointReachable(x, y) {
    const l1 = ROBOT_CONFIG.l1;
    const l2 = ROBOT_CONFIG.l2;
    const distance = Math.sqrt(x * x + y * y);
    
    // El punto debe estar dentro del área alcanzable
    return distance <= (l1 + l2) && distance >= Math.abs(l1 - l2);
}

function validateInput() {
    const x = parseFloat(xdInput.value);
    const y = parseFloat(ydInput.value);
    
    if (isNaN(x) || isNaN(y)) return;
    
    if (!isPointReachable(x, y)) {
        showAlert("¡Advertencia! El punto está fuera del espacio de trabajo del robot.");
    } else {
        hideAlert();
    }
}

// ==================== GENERACIÓN DE TRAYECTORIAS ====================
function generateTrajectory(qd1, qd2, ti = 0, tf = 20) {
    const timeSteps = 100;
    const trajectory = {
        time: [],
        q1: [],
        q2: [],
        x: [],
        y: []
    };
    
    const currentQ1 = robotState.q1;
    const currentQ2 = robotState.q2;
    
    // Polinomio de quinto orden: q(t) = a0 + a1*t + a2*t² + a3*t³ + a4*t⁴ + a5*t⁵
    // Condiciones: posición, velocidad y aceleración inicial/final = 0
    
    for (let i = 0; i <= timeSteps; i++) {
        const t = ti + (tf - ti) * (i / timeSteps);
        const normalizedTime = (t - ti) / (tf - ti);
        
        // Polinomio de quinto orden para transición suave
        const h = 10 * Math.pow(normalizedTime, 3) - 
                  15 * Math.pow(normalizedTime, 4) + 
                  6 * Math.pow(normalizedTime, 5);
        
        const q1 = currentQ1 + (qd1 - currentQ1) * h;
        const q2 = currentQ2 + (qd2 - currentQ2) * h;
        
        // Cinemática directa para la trayectoria cartesiana
        const x = ROBOT_CONFIG.l1 * Math.cos(q1) + ROBOT_CONFIG.l2 * Math.cos(q1 + q2);
        const y = ROBOT_CONFIG.l1 * Math.sin(q1) + ROBOT_CONFIG.l2 * Math.sin(q1 + q2);
        
        trajectory.time.push(t);
        trajectory.q1.push(q1);
        trajectory.q2.push(q2);
        trajectory.x.push(x);
        trajectory.y.push(y);
    }
    
    return trajectory;
}

// ==================== DIBUJO DEL ROBOT ====================
function drawRobot() {
    // Limpiar canvas
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    
    // Configuración de coordenadas (origen en el centro abajo)
    const originX = canvas.width / 2;
    const originY = canvas.height - 50;
    const scale = 1000; // píxeles por metro
    
    // Dibujar ejes coordenados
    drawCoordinateSystem(originX, originY, scale);
    
    // Calcular posiciones de las articulaciones
    const joint1X = originX + ROBOT_CONFIG.l1 * Math.cos(robotState.q1) * scale;
    const joint1Y = originY - ROBOT_CONFIG.l1 * Math.sin(robotState.q1) * scale;
    
    const x1 = originX + robotState.currentPos.x * scale;
    const y1 = originY - robotState.currentPos.y * scale;
    
    // Dibujar eslabones
    ctx.strokeStyle = '#007bff';
    ctx.lineWidth = 8;
    ctx.lineCap = 'round';
    
    // Eslabón 1
    ctx.beginPath();
    ctx.moveTo(originX, originY);
    ctx.lineTo(joint1X, joint1Y);
    ctx.stroke();
    
    // Eslabón 2
    ctx.beginPath();
    ctx.moveTo(joint1X, joint1Y);
    ctx.lineTo(x1, y1);
    ctx.stroke();
    
    // Dibujar pinza
    const gripperAngle = robotState.q1 + robotState.q2;
    const gripperLength = ROBOT_CONFIG.gripper * scale;
    const gripperEndX = x1 + gripperLength * Math.cos(gripperAngle);
    const gripperEndY = y1 - gripperLength * Math.sin(gripperAngle);
    
    ctx.strokeStyle = '#28a745';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(x1, y1);
    ctx.lineTo(gripperEndX, gripperEndY);
    ctx.stroke();
    
    // Dibujar articulaciones
    drawJoint(originX, originY, 12, '#6c757d'); // Base
    drawJoint(joint1X, joint1Y, 10, '#ffc107'); // Articulación 1
    drawJoint(x1, y1, 10, '#dc3545'); // TCP
    
    // Dibujar trayectoria deseada
    drawTrajectory(originX, originY, scale);
    
    // Etiqueta del TCP 
    ctx.fillStyle = '#333';
    ctx.font = '14px Arial';
    ctx.fillText(`TCP: (${robotState.currentPos.x.toFixed(3)}, ${robotState.currentPos.y.toFixed(3)})`, x1 + 10, y1 - 10);
}

function drawCoordinateSystem(originX, originY, scale) {
    ctx.strokeStyle = '#ccc';
    ctx.lineWidth = 1;
    
    // Eje X
    ctx.beginPath();
    ctx.moveTo(50, originY);
    ctx.lineTo(canvas.width - 50, originY);
    ctx.stroke();
    
    // Eje Y
    ctx.beginPath();
    ctx.moveTo(originX, canvas.height - 50);
    ctx.lineTo(originX, 50);
    ctx.stroke();
    
    // Etiquetas de ejes - CORREGIDO
    ctx.fillStyle = '#666';
    ctx.font = '12px Arial';
    ctx.fillText('x (m)', canvas.width - 40, originY - 10);
    ctx.fillText('y (m)', originX + 10, 40);
}

function drawJoint(x, y, radius, color) {
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.arc(x, y, radius, 0, 2 * Math.PI);
    ctx.fill();
    ctx.strokeStyle = '#333';
    ctx.lineWidth = 2;
    ctx.stroke();
}

function drawTrajectory(originX, originY, scale) {
    if (robotState.trajectory.length === 0) return;
    
    ctx.strokeStyle = '#6f42c1';
    ctx.lineWidth = 2;
    ctx.setLineDash([5, 5]);
    
    ctx.beginPath();
    robotState.trajectory.forEach((point, index) => {
        const x = originX + point.x * scale;
        const y = originY - point.y * scale;
        
        if (index === 0) {
            ctx.moveTo(x, y);
        } else {
            ctx.lineTo(x, y);
        }
    });
    ctx.stroke();
    ctx.setLineDash([]);
}

// ==================== ACTUALIZACIÓN DE GRÁFICAS ====================
function updateCharts(trajectory) {
    // Actualizar gráfica de q1
    q1Chart.data.labels = trajectory.time;
    q1Chart.data.datasets[0].data = trajectory.q1;
    q1Chart.update();
    
    // Actualizar gráfica de q2
    q2Chart.data.labels = trajectory.time;
    q2Chart.data.datasets[0].data = trajectory.q2;
    q2Chart.update();
}

// ==================== MANEJO DE EVENTOS ====================
function moveToPosition() {
    const xd = parseFloat(xdInput.value);
    const yd = parseFloat(ydInput.value);
    
    if (isNaN(xd) || isNaN(yd)) {
        showAlert("Por favor ingresa coordenadas válidas.");
        return;
    }
    
    if (!isPointReachable(xd, yd)) {
        showAlert("¡Error! El punto especificado está fuera del espacio de trabajo del robot.");
        return;
    }
    
    hideAlert();
    
    // Calcular cinemática inversa
    const ikSolution = inverseKinematics(xd, yd);
    if (!ikSolution) {
        showAlert("No se pudo calcular la cinemática inversa para esta posición.");
        return;
    }
    
    // Generar trayectoria
    const trajectory = generateTrajectory(ikSolution.q1, ikSolution.q2);
    robotState.trajectory = trajectory.x.map((x, i) => ({ x, y: trajectory.y[i] }));
    
    // Actualizar estado del robot (movimiento instantáneo)
    robotState.q1 = ikSolution.q1;
    robotState.q2 = ikSolution.q2;
    robotState.currentPos = { x: xd, y: yd };
    
    // Actualizar visualización
    updateCharts(trajectory);
    drawRobot();
    updateDisplay();
}

function goToHomePosition() {
    xdInput.value = ROBOT_CONFIG.initialPos.x;
    ydInput.value = ROBOT_CONFIG.initialPos.y;
    moveToPosition();
}

// ==================== UTILIDADES ====================
function showAlert(message) {
    alertMessage.textContent = message;
    alertMessage.style.display = 'block';
}

function hideAlert() {
    alertMessage.style.display = 'none';
}

function updateDisplay() {
    currentPosSpan.textContent = `x: ${robotState.currentPos.x.toFixed(3)}, y: ${robotState.currentPos.y.toFixed(3)}`;
    theta1Span.textContent = (robotState.q1 * 180 / Math.PI).toFixed(1);
    theta2Span.textContent = (robotState.q2 * 180 / Math.PI).toFixed(1);
}

// ==================== INICIAR APLICACIÓN ====================
document.addEventListener('DOMContentLoaded', init);

// Generar trayectoria inicial al cargar
window.addEventListener('load', function() {
    const trajectory = generateTrajectory(robotState.q1, robotState.q2);
    robotState.trajectory = trajectory.x.map((x, i) => ({ x, y: trajectory.y[i] }));
    updateCharts(trajectory);
    hideAlert(); // Ocultar mensaje de error inicial
});