require('dotenv').config({ path: './.env' });

const winston = require('winston');

const logger = winston.createLogger({
    level: 'info',
    format: winston.format.combine(
        winston.format.timestamp(),
        winston.format.json()
    ),
    defaultMeta: {},
    transports: [
        new winston.transports.File({ filename: './logs/server/error.log', level: 'error' }),
        new winston.transports.File({ filename: './logs/server/combined.log' }),
    ],
})
if (process.env.NODE_ENV !== 'production') {
    logger.add(new winston.transports.Console({
        format: winston.format.combine(
            winston.format.colorize(),
            winston.format.splat(),
            winston.format.simple()
        ),
    }));
}

logger.exceptions.handle(
    new winston.transports.File({ filename: './logs/exceptions.log' })
)

module.exports = logger;