## ---- dependencies
if (!require("data.table")){
    install.packages("data.table", repos="http://cran.rstudio.com/")
    library("data.table")
}

if (!require("stargazer")){
    install.packages("stargazer", repos="https://cran.rstudio.com/")
    library("stargazer")
}

# First we need to get data from each robot and combine into one data set
## ---- read_data
for (robot in 1:params$robots) {
    gazebo_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_gazebo_odom.csv")})
    gazebo_full_filenames <- lapply(gazebo_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    gazebo <- rbindlist(lapply(gazebo_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    discrete_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_discrete_filter_odom.csv")})
    discrete_full_filenames <- lapply(discrete_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    discrete <- rbindlist(lapply(discrete_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    continuous_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_continuous_filter_odom.csv")})
    continuous_full_filenames <- lapply(continuous_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    continuous <- rbindlist(lapply(continuous_full_filenames, FUN=function(file){fread(file, header=T, sep=",")}))

    external_filenames <- lapply(1:params$robots, FUN=function(robot){paste0("turtlebot", robot, "_external_pose_count.csv")})
    external_full_filenames <- lapply(external_filenames, FUN=function(filename){paste(params$data_dir, params$experiment, filename, sep="/")})
    external_data_tables <- lapply(external_full_filenames, FUN=function(file){fread(file, header=T, sep=",")})
}

## ---- calculations
# Now calculate errors
gazebo$dist_from_origin <- sqrt(gazebo$x_position ^ 2 + gazebo$y_position ^ 2)

discrete$x_error <- gazebo$x_position - discrete$x_position
discrete$y_error <- gazebo$y_position - discrete$y_position
discrete$horizontal_error <- sqrt(discrete$x_error ^ 2 + discrete$y_error ^ 2)

correct_yaw <- function(yaw) {
    while (yaw > pi) {
        yaw <- yaw - 2*pi
    }
    while (yaw < -1*pi) {
        yaw <- yaw + 2*pi
    }

    return(yaw)
}

vector_correct_yaw <- Vectorize(correct_yaw)
discrete$yaw_error <- vector_correct_yaw(gazebo$yaw - discrete$yaw)
continuous$yaw_error <- vector_correct_yaw(gazebo$yaw - continuous$yaw)

# for (i in 1:NROW(discrete$yaw)) {
#     yaw <- gazebo$yaw[i] - discrete$yaw[i]
#
#     while (yaw > pi) {
#         yaw <- yaw - 2*pi
#     }
#     while (yaw < -1*pi){
#         yaw <- yaw + 2*pi
#     }
#     discrete$yaw_error[i] <- yaw
# }

continuous$x_error <- gazebo$x_position - continuous$x_position
continuous$y_error <- gazebo$y_position - continuous$y_position
continuous$horizontal_error <- sqrt(continuous$x_error ^ 2 + continuous$y_error ^ 2)

# for (i in 1:NROW(continuous$yaw)) {
#     yaw <- gazebo$yaw[i] - continuous$yaw[i]
#
#     while (yaw > pi) {
#         yaw <- yaw - 2*pi
#     }
#     while (yaw < -1*pi){
#         yaw <- yaw + 2*pi
#     }
#     continuous$yaw_error[i] <- yaw
# }

external_data_averages <- lapply(external_data_tables, FUN=function(table){table$count[length(table)]})

## ---- plot
plot(gazebo$x_position, gazebo$y_position,
     main = "Ground truth visited locations of robots")

hist(gazebo$dist_from_origin,
     main = "Distance from origin vs. time")

hist(continuous$x_error,
     main = "Continuous x_error")

hist(continuous$y_error,
      main = "Continuous y_error")

hist(continuous$horizontal_error,
     main = "Continuous total distance error")

hist(discrete$x_error,
     main = "Discrete x_error")

hist(discrete$y_error,
     main = "Discrete y_error")

hist (discrete$horizontal_error,
      main = "Discrete total distance error")

## ---- summary
summary(continuous$x_error)
summary(continuous$y_error)
summary(continuous$yaw_error)
summary(continuous$horizontal_error)

summary(discrete$x_error)
summary(discrete$y_error)
summary(discrete$yaw_error)
summary(discrete$horizontal_error)

if (params$robot >= 2) {
    summary(external_data_averages)
}

## ---- external_figures
figure_dir <- "/home/matt/thesis/writing/r_figures/"
filename = paste0(figure_dir, params$experiment, "_continuous_error.pdf")
pdf(filename)
plot(continuous$horizontal_error, main="Continuous Filter Error", sub=paste0("For ", params$experiment, " Experiment"), xlab="Time (.1s)", ylab="Horizontal Position Error (m)")

dev.off()

filename = paste0(figure_dir, params$experiment, "_discrete_error.pdf")
pdf(filename)
plot(discrete$horizontal_error, main="Discrete Filter Error", sub=paste0("For ", params$experiment, " Experiment"), xlab="Time (.1s)", ylab="Horizontal Position Error (m)")
dev.off()

if (params$experiment == "one_stationary_noiseless") {
    gazebo$horizontal_error <- sqrt(gazebo$x_position ^ 2 + gazebo$y_position ^ 2)
    pdf(paste0(figure_dir, "gazebo_odom_drift.pdf"))

    plot(gazebo$horizontal_error, main="Gazebo Odometry Drift for Stationary Robot with Noiseless Odometry", ylab="Distance from Origin (m)", xlab="Time (.1s)")
    dev.off()
}
## ---- stargazer_tables
table_dir <- "/home/matt/thesis/writing/autogenerated_tables/"

out_file <- paste0(table_dir, params$experiment, "_continuous_summary.tex")
tex_label <- paste0("tab:", params$experiment, "_continuous_summary")
stargazer(continuous,
          out=out_file,
          table.placement="h",
          label=tex_label,
          title=gsub("_", "-", paste0("Continuous Filter Estimate for ", params$experiment, " Experiment")),
          digits.extra = 20)

out_file <- paste0(table_dir, params$experiment, "_discrete_summary.tex")
tex_label <- paste0("tab:", params$experiment, "_discrete_summary")
stargazer(discrete,
          out=out_file,
          table.placement="h",
          label=tex_label,
          title=gsub("_", "-", paste0("Discrete Filter Estimate for ", params$experiment, " Experiment")),
          digits.extra = 20)

if (params$experiment == "one_stationary_noiseless") {
    stargazer(gazebo,
              out=paste0(table_dir, "gazebo_stationary_noiseless_summary.tex"),
              table.placement="h",
              label="tab:gazebo_stationary_noiseless_summary",
              title="Ground Truth Noiseless Odometry for Stationary Robot located at Origin",
              digits.extra = 20)
}
