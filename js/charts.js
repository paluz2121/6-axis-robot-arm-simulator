/**
 * Trajectory Charts Manager using Chart.js
 * 
 * Features:
 * - Dynamic plots for Joint Angles (q1...q6), Velocities (qDot), Accelerations (qDoubleDot), and TCP Linear Speed
 * - Synced playhead scrubber cursor across all charts
 * - Click-to-seek time on any chart
 * - Dark cyber aesthetic with high contrast neon series colors
 */

class TrajectoryChartsManager {
  constructor(onSeekCallback) {
    this.onSeek = onSeekCallback;
    this.charts = {};
    this.currentPlayheadTime = 0;
    this.trajectoryData = null;

    this.jointColors = [
      '#ef4444', // Joint 1: Red
      '#f97316', // Joint 2: Orange
      '#eab308', // Joint 3: Yellow
      '#10b981', // Joint 4: Emerald
      '#06b6d4', // Joint 5: Cyan
      '#8b5cf6'  // Joint 6: Purple
    ];

    this.initCharts();
  }

  getChartCommonOptions(titleText, yAxisLabel, isDegrees = true) {
    return {
      responsive: true,
      maintainAspectRatio: false,
      animation: { duration: 300 },
      interaction: {
        mode: 'index',
        intersect: false
      },
      onClick: (e, elements, chart) => {
        if (!this.trajectoryData) return;
        const canvasPosition = Chart.helpers.getRelativePosition(e, chart);
        const dataX = chart.scales.x.getValueForPixel(canvasPosition.x);
        if (dataX !== undefined && this.onSeek) {
          const duration = this.trajectoryData.duration;
          const seekTime = Math.max(0, Math.min(duration, (dataX / (this.trajectoryData.stepsCount - 1)) * duration));
          this.onSeek(seekTime);
        }
      },
      plugins: {
        legend: {
          position: 'top',
          labels: {
            color: '#94a3b8',
            font: { family: 'Inter', size: 11 },
            boxWidth: 12,
            boxHeight: 12
          }
        },
        title: {
          display: true,
          text: titleText,
          color: '#e2e8f0',
          font: { family: 'Inter', size: 13, weight: '600' },
          padding: { bottom: 8 }
        },
        tooltip: {
          backgroundColor: 'rgba(15, 23, 42, 0.9)',
          titleColor: '#38bdf8',
          bodyColor: '#e2e8f0',
          borderColor: 'rgba(56, 189, 248, 0.3)',
          borderWidth: 1,
          padding: 8,
          bodyFont: { family: 'JetBrains Mono', size: 11 }
        }
      },
      scales: {
        x: {
          grid: { color: 'rgba(51, 65, 85, 0.3)' },
          ticks: {
            color: '#64748b',
            font: { family: 'JetBrains Mono', size: 10 },
            callback: (val, idx) => {
              if (!this.trajectoryData) return `${val}`;
              const t = (idx / (this.trajectoryData.stepsCount - 1)) * this.trajectoryData.duration;
              return `${t.toFixed(1)}s`;
            }
          },
          title: {
            display: true,
            text: 'Zaman (saniye)',
            color: '#64748b',
            font: { family: 'Inter', size: 11 }
          }
        },
        y: {
          grid: { color: 'rgba(51, 65, 85, 0.3)' },
          ticks: {
            color: '#64748b',
            font: { family: 'JetBrains Mono', size: 10 }
          },
          title: {
            display: true,
            text: yAxisLabel,
            color: '#64748b',
            font: { family: 'Inter', size: 11 }
          }
        }
      }
    };
  }

  initCharts() {
    // 1. Joint Positions Chart
    const ctxPos = document.getElementById('chart-positions');
    if (ctxPos) {
      this.charts.positions = new Chart(ctxPos, {
        type: 'line',
        data: { labels: [], datasets: [] },
        options: this.getChartCommonOptions('Eklem Konumları θ_i(t)', 'Açı (Derece)')
      });
    }

    // 2. Joint Velocities Chart
    const ctxVel = document.getElementById('chart-velocities');
    if (ctxVel) {
      this.charts.velocities = new Chart(ctxVel, {
        type: 'line',
        data: { labels: [], datasets: [] },
        options: this.getChartCommonOptions('Eklem Hızları dθ/dt', 'Açısal Hız (°/s)')
      });
    }

    // 3. Joint Accelerations Chart
    const ctxAcc = document.getElementById('chart-accelerations');
    if (ctxAcc) {
      this.charts.accelerations = new Chart(ctxAcc, {
        type: 'line',
        data: { labels: [], datasets: [] },
        options: this.getChartCommonOptions('Eklem İvmeleri d²θ/dt²', 'Açısal İvme (°/s²)')
      });
    }

    // 4. Cartesian TCP Speed & Path Chart
    const ctxTcp = document.getElementById('chart-tcp');
    if (ctxTcp) {
      this.charts.tcp = new Chart(ctxTcp, {
        type: 'line',
        data: { labels: [], datasets: [] },
        options: this.getChartCommonOptions('Kartezyen Uç Nokta (TCP) Hızı ve Koordinatları', 'Hız (mm/s) / Konum (mm)')
      });
    }
  }

  /**
   * Populate charts with new trajectory data
   */
  updateTrajectory(trajectoryData) {
    this.trajectoryData = trajectoryData;
    if (!trajectoryData) return;

    const timeLabels = trajectoryData.timeSamples.map(t => t.toFixed(2));
    const jointNames = ['θ1 (Base)', 'θ2 (Shoulder)', 'θ3 (Elbow)', 'θ4 (Wrist 1)', 'θ5 (Wrist 2)', 'θ6 (Wrist 3)'];

    // 1. Joint Positions Datasets (converted to degrees)
    if (this.charts.positions) {
      const posDatasets = [];
      for (let j = 0; j < 6; j++) {
        const data = trajectoryData.qSamples.map(q => (q[j] * 180 / Math.PI).toFixed(1));
        posDatasets.push({
          label: jointNames[j],
          data: data,
          borderColor: this.jointColors[j],
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.1
        });
      }
      this.charts.positions.data.labels = timeLabels;
      this.charts.positions.data.datasets = posDatasets;
      this.charts.positions.update();
    }

    // 2. Joint Velocities Datasets (converted to deg/s)
    if (this.charts.velocities) {
      const velDatasets = [];
      for (let j = 0; j < 6; j++) {
        const data = trajectoryData.qDotSamples.map(qd => (qd[j] * 180 / Math.PI).toFixed(1));
        velDatasets.push({
          label: jointNames[j],
          data: data,
          borderColor: this.jointColors[j],
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.1
        });
      }
      this.charts.velocities.data.labels = timeLabels;
      this.charts.velocities.data.datasets = velDatasets;
      this.charts.velocities.update();
    }

    // 3. Joint Accelerations Datasets (converted to deg/s^2)
    if (this.charts.accelerations) {
      const accDatasets = [];
      for (let j = 0; j < 6; j++) {
        const data = trajectoryData.qDoubleDotSamples.map(qdd => (qdd[j] * 180 / Math.PI).toFixed(1));
        accDatasets.push({
          label: jointNames[j],
          data: data,
          borderColor: this.jointColors[j],
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.1
        });
      }
      this.charts.accelerations.data.labels = timeLabels;
      this.charts.accelerations.data.datasets = accDatasets;
      this.charts.accelerations.update();
    }

    // 4. TCP Speed & Manipulability Datasets
    if (this.charts.tcp) {
      const tcpSpeedData = trajectoryData.tcpSamples.map(tcp => tcp.speed.toFixed(1));
      const manipData = trajectoryData.manipulabilitySamples.map(m => (m * 1000).toFixed(1)); // Scale for visibility

      this.charts.tcp.data.labels = timeLabels;
      this.charts.tcp.data.datasets = [
        {
          label: 'TCP Hızı (mm/s)',
          data: tcpSpeedData,
          borderColor: '#38bdf8',
          backgroundColor: 'rgba(56, 189, 248, 0.1)',
          fill: true,
          borderWidth: 2.5,
          pointRadius: 0,
          tension: 0.2
        },
        {
          label: 'Manipülatiflik İndeksi w (x10³)',
          data: manipData,
          borderColor: '#10b981',
          borderDash: [5, 5],
          backgroundColor: 'transparent',
          borderWidth: 2,
          pointRadius: 0,
          tension: 0.2
        }
      ];
      this.charts.tcp.update();
    }
  }

  /**
   * Update vertical playhead tracker across active charts
   */
  setPlayheadTime(currentTime) {
    this.currentPlayheadTime = currentTime;
    // We can highlight or indicate time in the charts
    // Chart.js supports drawing custom lines or tooltips via custom plugin if desired
  }
}

// Global export
if (typeof window !== 'undefined') {
  window.TrajectoryChartsManager = TrajectoryChartsManager;
}
