FROM eclipse-temurin:11-jdk AS build
WORKDIR /src
COPY mvnw pom.xml ./
COPY .mvn .mvn
RUN chmod +x mvnw && ./mvnw -B -q dependency:go-offline
COPY src src
COPY config config
COPY samples samples
RUN ./mvnw -B -q package -DskipTests

FROM eclipse-temurin:11-jre
WORKDIR /app
RUN mkdir -p /data /app/config /app/samples
COPY --from=build /src/target/heatnet.jar /app/app.jar
COPY docker-entrypoint.sh /app/docker-entrypoint.sh
COPY config /app/config
COPY samples /app/samples
COPY ["!!!_Датасет.geojson", "/app/samples/contest-input.geojson"]
RUN chmod +x /app/docker-entrypoint.sh
EXPOSE 8080
ENV JAVA_OPTS="-Xms512m -Xmx12g -XX:+UseG1GC"
ENTRYPOINT ["/app/docker-entrypoint.sh"]
